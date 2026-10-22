#ifndef ATG_ENGINE_SIM_HOST_CONTROL_SERVER_H
#define ATG_ENGINE_SIM_HOST_CONTROL_SERVER_H

#include "host_settings.h"

#include <nlohmann/json.hpp>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

class EngineHost;

// Local control socket: one JSON object per line in, one per line out.
//
//   {"cmd":"status"}                                   engine, bridge and performance state
//   {"cmd":"get_settings"}
//   {"cmd":"set","settings":{"bridge":{"volume":0.2}}} change settings (partial), saved to disk
//   {"cmd":"load","script":"/abs/path/engine.mr"}      load an engine ("" for the default)
//   {"cmd":"retune"}                                   re-profile the engine for BeamNG
//   {"cmd":"control","ignition":true,"starter":false,"throttle":0.2}
//   {"cmd":"shutdown"}                                 ask engine-sim to exit
//   {"cmd":"idle_air","percent":2.5}                 air past the closed throttle (or "reset":true)
//   {"cmd":"generate"}                                 build and load an engine matching the BeamNG car
//   {"cmd":"synth","volume":0.8,"convolution":1,...}   engine sound mix (per engine, not saved);
//                                                      "reset":true goes back to the engine's own
//
// Replies carry "ok" (and "error" when false), plus "id" if the request had one.
// Try it from a shell:  echo '{"cmd":"status"}' | socat - UNIX-CONNECT:$XDG_RUNTIME_DIR/engine-sim.sock
class ControlServer {
public:
    ControlServer(EngineHost *host, HostSettings settings, std::string settingsFile);
    ~ControlServer();

    ControlServer(const ControlServer &) = delete;
    ControlServer &operator=(const ControlServer &) = delete;

    // $XDG_RUNTIME_DIR/engine-sim.sock, or /tmp/engine-sim-<uid>.sock without XDG_RUNTIME_DIR
    static std::string defaultSocketPath();

    // Fails if another engine-sim is already listening on the path
    bool start(const std::string &socketPath, std::string *error);
    void stop();

    const std::string &socketPath() const { return m_socketPath; }

    // Called (on the server thread) for "shutdown"; set before start()
    void setShutdownHandler(std::function<void()> handler) { m_shutdownHandler = std::move(handler); }

    HostSettings settings() {
        std::lock_guard<std::mutex> lock(m_settingsMutex);
        return m_settings;
    }

    // Handles one request (also used directly by tests)
    nlohmann::json handle(const nlohmann::json &request);

private:
    void serverThread();

    nlohmann::json status();
    nlohmann::json setSettings(const nlohmann::json &request);
    nlohmann::json load(const nlohmann::json &request);
    nlohmann::json control(const nlohmann::json &request);
    nlohmann::json synth(const nlohmann::json &request);
    nlohmann::json generate();
    void maybeAutoGenerate();
    nlohmann::json idleAir(const nlohmann::json &request);
    nlohmann::json buildEngine();

    // Loads a script on the host's owner thread and waits; records it as the saved engine
    nlohmann::json loadAndRemember(const std::string &script);
    void saveSettings();

private:
    EngineHost *m_host;
    std::string m_lastAutoCar; // The car the automatic engine was last built for

    // The latest engine build's outcome, shown by the panel however it was started
    int m_buildId = 0;
    bool m_buildOk = false;
    std::string m_buildMessage;
    std::function<void()> m_shutdownHandler;
    std::string m_settingsFile;

    std::mutex m_settingsMutex;
    HostSettings m_settings;

    std::string m_socketPath;
    int m_listenSocket = -1;
    std::atomic<bool> m_running{false};
    std::thread m_thread;
};

#endif /* ATG_ENGINE_SIM_HOST_CONTROL_SERVER_H */
