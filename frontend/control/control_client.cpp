#include "control_client.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

using nlohmann::json;
using clock_type = std::chrono::steady_clock;

ControlClient::ControlClient(std::string socketPath) : m_socketPath(std::move(socketPath)) {
    m_thread = std::thread(&ControlClient::run, this);
}

ControlClient::~ControlClient() {
    m_running = false;
    m_wake.notify_all();
    if (m_thread.joinable()) m_thread.join();
    disconnect();
}

std::string ControlClient::defaultSocketPath() {
    const char *runtime = std::getenv("XDG_RUNTIME_DIR");
    if (runtime != nullptr && runtime[0] != '\0') return std::string(runtime) + "/engine-sim.sock";
    return "/tmp/engine-sim-" + std::to_string(getuid()) + ".sock";
}

void ControlClient::send(json request) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push_back(std::move(request));
    }
    m_wake.notify_all();
}

ControlClient::Snapshot ControlClient::snapshot() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_snapshot;
}

bool ControlClient::connectSocket() {
    sockaddr_un address{};
    if (m_socketPath.size() >= sizeof(address.sun_path)) return false;
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, m_socketPath.c_str(), m_socketPath.size() + 1);

    const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return false;
    if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
        close(fd);
        return false;
    }

    m_fd = fd;
    m_readBuffer.clear();
    return true;
}

void ControlClient::disconnect() {
    if (m_fd >= 0) close(m_fd);
    m_fd = -1;
}

bool ControlClient::request(const json &request, json *reply, int timeoutMs) {
    const std::string line = request.dump() + "\n";
    size_t sent = 0;
    while (sent < line.size()) {
        const ssize_t n = ::send(m_fd, line.data() + sent, line.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }

    const auto deadline = clock_type::now() + std::chrono::milliseconds(timeoutMs);
    for (;;) {
        const size_t newline = m_readBuffer.find('\n');
        if (newline != std::string::npos) {
            *reply = json::parse(m_readBuffer.substr(0, newline), nullptr, false);
            m_readBuffer.erase(0, newline + 1);
            return !reply->is_discarded();
        }

        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - clock_type::now()).count();
        if (left <= 0) return false;

        pollfd waitFor{ m_fd, POLLIN, 0 };
        if (poll(&waitFor, 1, static_cast<int>(left)) <= 0) continue;

        char chunk[8192];
        const ssize_t n = recv(m_fd, chunk, sizeof(chunk), 0);
        if (n <= 0) return false;
        m_readBuffer.append(chunk, static_cast<size_t>(n));
    }
}

void ControlClient::run() {
    auto nextStatus = clock_type::now();

    while (m_running) {
        if (m_fd < 0 && !connectSocket()) {
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_snapshot.connected = false;
                m_snapshot.status = nullptr;
                m_queue.clear(); // Changes made while disconnected would apply to a stranger
            }
            std::unique_lock<std::mutex> lock(m_mutex);
            m_wake.wait_for(lock, std::chrono::milliseconds(500), [this] { return !m_running; });
            continue;
        }

        // Queued requests first, so the next status reflects them
        for (;;) {
            json next;
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (m_queue.empty()) break;
                next = std::move(m_queue.front());
                m_queue.pop_front();
                m_snapshot.busy = true;
            }

            const bool slow = next.value("cmd", "") == "load";
            json reply;
            const bool ok = request(next, &reply, slow ? 65000 : 3000);

            std::lock_guard<std::mutex> lock(m_mutex);
            m_snapshot.busy = false;
            if (!ok) {
                disconnect();
                break;
            }
            if (!reply.value("ok", false)) {
                m_snapshot.message = reply.value("error", std::string("request failed"));
                if (reply.contains("errors")) m_snapshot.message += "\n" + reply["errors"].get<std::string>();
                m_snapshot.messageIsError = true;
            }
            else if (reply.contains("message")) {
                m_snapshot.message = reply["message"].get<std::string>();
                m_snapshot.messageIsError = false;
            }
        }
        if (m_fd < 0) continue;

        if (clock_type::now() >= nextStatus) {
            json reply;
            if (!request({ { "cmd", "status" } }, &reply, 3000)) {
                disconnect();
                continue;
            }

            std::lock_guard<std::mutex> lock(m_mutex);
            m_snapshot.connected = true;
            m_snapshot.status = std::move(reply);
            nextStatus = clock_type::now() + std::chrono::milliseconds(100);
        }

        std::unique_lock<std::mutex> lock(m_mutex);
        m_wake.wait_until(lock, nextStatus, [this] { return !m_running || !m_queue.empty(); });
    }
}
