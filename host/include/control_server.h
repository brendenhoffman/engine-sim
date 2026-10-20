#ifndef ATG_ENGINE_SIM_HOST_CONTROL_SERVER_H
#define ATG_ENGINE_SIM_HOST_CONTROL_SERVER_H

#include "host_settings.h"

#include <nlohmann/json.hpp>

#include <atomic>
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
    void saveSettings();

private:
    EngineHost *m_host;
    std::string m_settingsFile;

    std::mutex m_settingsMutex;
    HostSettings m_settings;

    std::string m_socketPath;
    int m_listenSocket = -1;
    std::atomic<bool> m_running{false};
    std::thread m_thread;
};

#endif /* ATG_ENGINE_SIM_HOST_CONTROL_SERVER_H */
