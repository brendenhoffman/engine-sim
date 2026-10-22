#ifndef ATG_ENGINE_SIM_HOST_BEAM_BRIDGE_H
#define ATG_ENGINE_SIM_HOST_BEAM_BRIDGE_H

#include "bridge_transport.h"

#include <atomic>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

class Engine;
class Simulator;

// Drives the loaded engine from BeamNG: a held dynamometer follows BeamNG's rpm, throttle and
// ignition come from the game, and the torque engine-sim measures goes back to the car.
// Optionally profiles the engine (a WOT and closed-throttle sweep) so the car can be retuned
// to match it. Ported from BeamEngineSim.
//
// Everything except active()/linked() runs under the EngineHost lock on the simulation thread.
class BeamBridge {
public:
    struct Settings {
        bool connect = true;            // Follow BeamNG when it's sending
        bool driveCar = false;          // Replace the car's engine torque with engine-sim's (off: sound only)
        bool matchCarPower = true;      // ...scaled to the car's own torque curve, so it keeps its stock power
        float volume = 0.20f;           // Gain of the audio sent to BeamNG (BeamEngineSim used 0.10 without headroom)
        float beamIdleRpm = 1100.0f;    // Below this, BeamNG keeps its own torque (steadier idle)
        bool tuneBeamEngine = false;    // Profile the engine and retune the car to match
        bool neutralEq = true;          // Flatten the car's sound EQ (tuned for its own samples) while linked
        // Level of engine-sim in the car's intake (engine bay) and exhaust sound sources, 0 to 1.
        // engine-sim models exhaust sound, so at full level the intake copy (right in front of
        // the camera in most cars) masks the exhaust's character
        float intakeLevel = 0.35f;
        float exhaustLevel = 1.0f;
        bool keepGameLowCut = false;
        bool autoEngine = false;        // Build and load a matching engine whenever the car changes
        bool gameBurble = true;         // The game's sustained afterfire burble (its bangs always play)
        // At idle, engine-sim's crank follows BeamNG's speed only this quickly (Hz), so its own
        // firing-to-firing unevenness swings the revs as in a free-running idle (0: rigid hold)
        float idleFollowHz = 2.3f;
        // How the idle works in game: 0 the engine's own choice, 1 follow the game's idle,
        // 2 idle freely as engine-sim does on its own (steps in only if it strays 300+ rpm)
        int idleMode = 0;
    };

    explicit BeamBridge(BridgeTransport *transport);

    Settings &settings() { return m_settings; }
    void retune();

    // Engine changed: forget the profile and re-link from the new engine's speed
    void reset();

    // Before each simulation tick
    void beforeFrame(Simulator *simulator, Engine *engine, double dt);
    // After each simulation step while linked: ramp the dyno across the tick
    void onStep(Simulator *simulator, int step, int iterationCount);
    // Audio produced this tick
    void onAudio(const int16_t *samples, int count);

    bool active() const { return m_active; }       // Following BeamNG right now
    bool tuning() const { return m_tuning; }       // Running the profile sweep

    // A status line worth showing ("BEAMNG BRIDGE CONNECTED"), or empty
    std::string takeLogMessage();

private:
    void deactivate(Simulator *simulator, Engine *engine);
    void runProfileSweep(Engine *engine, Simulator *simulator, double dt, double *rpm, double *throttle);
    void finishProfile(Engine *engine);
    void refreshVehicle();
    double beamTorqueAt(double rpm) const;
    double calibrationAt(double rpm, double limitRpm) const;

private:
    BridgeTransport *m_transport;
    Settings m_settings;

    std::atomic<bool> m_active{false};
    std::atomic<bool> m_tuning{false};
    bool m_retune = false;

    BridgeTransport::Input m_input;
    double m_rampFrom = 0;
    double m_rampTo = 0;
    std::deque<std::pair<double, double>> m_dynoHistory;
    std::deque<std::pair<double, double>> m_spinHistory; // (time, crank acceleration we imposed after it)
    double m_lastFrameTime = 0;

    int m_profileStep = 0;
    double m_profileTimer = 0;
    double m_profileSendTimer = 0;
    double m_torqueSendTimer = 0;
    bool m_profileValid = false;
    float m_profileId = 0;
    std::vector<float> m_profileWot, m_profileClosed, m_profilePacket;

    // The BeamNG car's own full-throttle curve, and car torque / engine-sim torque at each
    // profile point once measured
    std::string m_vehicleKey;
    std::vector<std::pair<double, double>> m_beamTorque;
    double m_beamInertia = 0;
    std::vector<double> m_calibration;
    double m_profileLimitRpm = 0;
    double m_vehicleTimer = 0;
    double m_beamIdleRpm = 0;       // The car's own idle speed, from its description

    // Smooth follower of BeamNG's rpm: packets arrive at its frame rate, and extrapolating
    // between them overshot on fast changes (engine braking), then stepped back
    double m_followRpm = -1;
    double m_followRate = 0;

    // Idle coupling: a slow speed controller in place of the rigid hold
    bool m_idleCoupling = false;
    double m_couplingIntegral = 0;
    double m_couplingTarget = 0; // rad/s
    bool m_idleFree = false;     // Free-running idle: the coupling only acts outside a band

    std::string m_logMessage;
};

#endif /* ATG_ENGINE_SIM_HOST_BEAM_BRIDGE_H */
