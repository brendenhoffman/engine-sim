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
        float volume = 0.20f;           // Gain of the audio sent to BeamNG (BeamEngineSim used 0.10 without headroom)
        float beamIdleRpm = 1100.0f;    // Below this, BeamNG keeps its own torque (steadier idle)
        bool tuneBeamEngine = false;    // Profile the engine and retune the car to match
        bool neutralEq = true;          // Flatten the car's sound EQ (tuned for its own samples) while linked
        bool intakeSound = true;        // Play engine-sim through the car's intake (engine) sound event
        bool exhaustSound = true;       // ...and through its exhaust sound event
        bool keepGameLowCut = false;    // Keep the game's ~90-190 Hz high-pass on engine sound (thins engine-sim out)
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

    int m_profileStep = 0;
    double m_profileTimer = 0;
    double m_profileSendTimer = 0;
    double m_torqueSendTimer = 0;
    bool m_profileValid = false;
    float m_profileId = 0;
    std::vector<float> m_profileWot, m_profileClosed, m_profilePacket;

    std::string m_logMessage;
};

#endif /* ATG_ENGINE_SIM_HOST_BEAM_BRIDGE_H */
