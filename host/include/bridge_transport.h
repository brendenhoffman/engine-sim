#ifndef ATG_ENGINE_SIM_HOST_BRIDGE_TRANSPORT_H
#define ATG_ENGINE_SIM_HOST_BRIDGE_TRANSPORT_H

#include <nlohmann/json.hpp>

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <netinet/in.h>
#include <thread>

// UDP link to BeamNG and its FMOD audio plugin, wire-compatible with BeamEngineSim.
// All values are little-endian float32 after a 4-byte tag.
//
//   BeamNG -> engine-sim (port 4570)
//     "BES1" rpm, throttle, ignition, volume, pan    vehicle state, ~60 Hz
//     "BESH"                                          heartbeat while the game is paused
//     "BESV" {json}                                   the player's car and engine, every ~3 s
//
//   engine-sim -> BeamNG (reply to the sender of BES1)
//     "BES1" torque_nm, rpm, boost_kpa     torque for BeamNG's engine; manifold pressure above
//                                         ambient (negative: vacuum) for its boost gauge
//     "BESN" (same layout)      keep BeamNG's own torque
//     "BESP" values...          engine profile for retuning the car, ~2 Hz
//     "BESC" 1, neutral_eq, intake_level, exhaust_level, profiling   sound settings for the mod, ~2 Hz
//
//   engine-sim -> FMOD plugin (port 4571)
//     "BESF" samples...         mono 44.1 kHz audio
//     "BESX" 1, keep_low_cut    plugin settings, ~2 Hz
//
// New message types get new tags; receivers ignore tags they don't know.
class BridgeTransport {
public:
    static constexpr int DefaultPort = 4570;
    static constexpr int DefaultAudioPort = 4571;

    // ENGINE_SIM_BRIDGE_PORT / ENGINE_SIM_AUDIO_PORT, else the defaults (tests use these to
    // stay clear of a running game)
    static int portFromEnvironment();
    static int audioPortFromEnvironment();

    struct Input {
        float rpm = 0;
        float throttle = 0;
        float ignition = 0;
        float volume = 0;
        float pan = 0;
    };

    BridgeTransport();
    ~BridgeTransport();

    BridgeTransport(const BridgeTransport &) = delete;
    BridgeTransport &operator=(const BridgeTransport &) = delete;

    // Binds 127.0.0.1:port and starts the receive and audio threads
    bool start(int port = DefaultPort, int audioPort = DefaultAudioPort);
    void stop();
    bool started() const { return m_socket >= 0; }

    // Latest vehicle state. smoothRpm extrapolates from the last two packets, at most one
    // packet interval ahead. Returns false if nothing has been received yet.
    bool latest(Input *input, double *ageSeconds, float *smoothRpm);

    // Appends each newly seen engine's description to this file (JSON lines). Set before start().
    void setVehicleLog(std::string path) { m_vehicleLogPath = std::move(path); }

    // Latest description of the player's car ("BESV"); false if none has arrived
    bool vehicleInfo(nlohmann::json *info, double *ageSeconds);

    // Least-squares slope of BeamNG's rpm over the last `windowSeconds`, in rpm/s
    double rpmRate(double windowSeconds);

    // NaN torque sends "keep BeamNG's torque"
    void sendTorque(float torqueNm, float rpm, float boostKpa = 0);
    void sendProfile(const float *values, int count);
    void sendSoundConfig(bool neutralEq, float intakeLevel, float exhaustLevel, bool profiling, bool gameBurble = true);
    void sendPluginConfig(bool keepGameLowCut);

    // Queues audio for the plugin; a sender thread paces it out at 44.1 kHz
    void sendAudio(const int16_t *samples, int count, float gain);

private:
    void receiveThread();
    void audioThread();
    bool peer(sockaddr_in *out);

private:
    int m_socket = -1;
    int m_audioPort = DefaultAudioPort;
    std::atomic<bool> m_running{false};
    std::thread m_receiver;
    std::thread m_audioSender;

    std::mutex m_inputMutex;
    sockaddr_in m_peer{};
    bool m_hasPeer = false;
    Input m_latest;
    double m_latestTime = -1;

    std::string m_vehicleLogPath;
    std::string m_lastLoggedEngine;
    nlohmann::json m_vehicleInfo;
    double m_vehicleInfoTime = -1;

    static constexpr int HistorySize = 64;
    struct Sample { double time; float rpm; };
    Sample m_history[HistorySize]{};
    int m_historyCount = 0;

    std::mutex m_audioMutex;
    std::deque<float> m_audioQueue;
};

#endif /* ATG_ENGINE_SIM_HOST_BRIDGE_TRANSPORT_H */
