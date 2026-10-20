#ifndef ENGINE_SIM_CONTROL_CLIENT_H
#define ENGINE_SIM_CONTROL_CLIENT_H

#include <nlohmann/json.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

// Talks to a running engine-sim over its control socket from a background thread, so slow
// requests (loading an engine) never stall the UI. Reconnects on its own; polls status.
class ControlClient {
public:
    struct Snapshot {
        bool connected = false;
        nlohmann::json status;          // Last "status" reply (engine, bridge, performance, settings)
        std::string message;            // Last notable reply ("Loaded GM LS", errors)
        bool messageIsError = false;
        bool busy = false;              // A request is in flight
    };

    explicit ControlClient(std::string socketPath);
    ~ControlClient();

    ControlClient(const ControlClient &) = delete;
    ControlClient &operator=(const ControlClient &) = delete;

    // $XDG_RUNTIME_DIR/engine-sim.sock (same rule as engine-sim itself)
    static std::string defaultSocketPath();

    // Queues a request. Replies with "ok": false, and replies to "load", show up in the snapshot.
    void send(nlohmann::json request);

    // Convenience for partial settings changes
    void setSettings(nlohmann::json settings) { send({ { "cmd", "set" }, { "settings", std::move(settings) } }); }

    Snapshot snapshot();

private:
    void run();
    bool connectSocket();
    void disconnect();
    bool request(const nlohmann::json &request, nlohmann::json *reply, int timeoutMs);

private:
    std::string m_socketPath;
    int m_fd = -1;
    std::string m_readBuffer;

    std::atomic<bool> m_running{true};
    std::thread m_thread;

    std::mutex m_mutex;
    std::condition_variable m_wake;
    std::deque<nlohmann::json> m_queue;
    Snapshot m_snapshot;
};

#endif /* ENGINE_SIM_CONTROL_CLIENT_H */
