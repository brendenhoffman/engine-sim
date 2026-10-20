#include "bridge_transport.h"

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {
    double nowSeconds() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    sockaddr_in localhost(int port) {
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        return addr;
    }
}

BridgeTransport::BridgeTransport() {
    /* void */
}

BridgeTransport::~BridgeTransport() {
    stop();
}

bool BridgeTransport::start(int port, int audioPort) {
    if (m_socket >= 0) return true;

    const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return false;

    const sockaddr_in addr = localhost(port);
    if (bind(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) != 0) {
        close(fd);
        return false;
    }

    m_socket = fd;
    m_audioPort = audioPort;
    m_running = true;
    m_receiver = std::thread(&BridgeTransport::receiveThread, this);
    m_audioSender = std::thread(&BridgeTransport::audioThread, this);

    return true;
}

void BridgeTransport::stop() {
    if (m_socket < 0) return;

    m_running = false;
    if (m_receiver.joinable()) m_receiver.join();
    if (m_audioSender.joinable()) m_audioSender.join();

    close(m_socket);
    m_socket = -1;
}

void BridgeTransport::receiveThread() {
    char buffer[64];
    pollfd waitFor{ m_socket, POLLIN, 0 };

    while (m_running) {
        // Wake up regularly so stop() doesn't hang on a quiet socket
        if (poll(&waitFor, 1, 100) <= 0) continue;

        sockaddr_in from{};
        socklen_t fromLength = sizeof(from);
        const ssize_t n = recvfrom(m_socket, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr *>(&from), &fromLength);
        if (n < 4) continue;

        if (n == 4 && std::memcmp(buffer, "BESH", 4) == 0) {
            // Paused game: keeps a live link alive, never starts one
            std::lock_guard<std::mutex> lock(m_inputMutex);
            const double now = nowSeconds();
            if (m_latestTime >= 0 && now - m_latestTime < 0.5) m_latestTime = now;
            continue;
        }

        if (n != 4 + static_cast<ssize_t>(sizeof(Input)) || std::memcmp(buffer, "BES1", 4) != 0) continue;

        Input input;
        std::memcpy(&input, buffer + 4, sizeof(input));
        const double now = nowSeconds();

        std::lock_guard<std::mutex> lock(m_inputMutex);
        m_history[m_historyCount++ % HistorySize] = { now, input.rpm };
        m_latest = input;
        m_latestTime = now;
        m_peer = from;
        m_hasPeer = true;
    }
}

bool BridgeTransport::latest(Input *input, double *ageSeconds, float *smoothRpm) {
    std::lock_guard<std::mutex> lock(m_inputMutex);
    if (m_latestTime < 0) return false;

    *input = m_latest;
    const double now = nowSeconds();
    *ageSeconds = now - m_latestTime;

    *smoothRpm = m_latest.rpm;
    if (m_historyCount >= 2) {
        const Sample &b = m_history[(m_historyCount - 1) % HistorySize];
        const Sample &a = m_history[(m_historyCount - 2) % HistorySize];
        const double span = b.time - a.time;
        if (span > 0.001 && span < 0.1) {
            const double f = std::min((now - a.time) / span, 2.0);
            *smoothRpm = static_cast<float>(a.rpm + (b.rpm - a.rpm) * f);
        }
    }

    return true;
}

double BridgeTransport::rpmRate(double windowSeconds) {
    std::lock_guard<std::mutex> lock(m_inputMutex);

    const double from = nowSeconds() - windowSeconds;
    double n = 0, st = 0, sr = 0, stt = 0, str = 0;
    for (int i = 1; i <= std::min(m_historyCount, HistorySize); ++i) {
        const Sample &p = m_history[(m_historyCount - i) % HistorySize];
        if (p.time < from) break;

        const double t = p.time - from;
        n += 1; st += t; sr += p.rpm; stt += t * t; str += t * p.rpm;
    }

    const double d = n * stt - st * st;
    return (n >= 3 && d > 0) ? (n * str - st * sr) / d : 0;
}

bool BridgeTransport::peer(sockaddr_in *out) {
    std::lock_guard<std::mutex> lock(m_inputMutex);
    *out = m_peer;
    return m_hasPeer;
}

void BridgeTransport::sendTorque(float torqueNm, float rpm) {
    sockaddr_in to;
    if (m_socket < 0 || !peer(&to)) return;

    char buffer[12];
    std::memcpy(buffer, std::isnan(torqueNm) ? "BESN" : "BES1", 4);
    std::memcpy(buffer + 4, &torqueNm, 4);
    std::memcpy(buffer + 8, &rpm, 4);
    sendto(m_socket, buffer, sizeof(buffer), 0, reinterpret_cast<const sockaddr *>(&to), sizeof(to));
}

void BridgeTransport::sendProfile(const float *values, int count) {
    sockaddr_in to;
    if (m_socket < 0 || !peer(&to) || count < 0 || count > 256) return;

    char buffer[4 + 4 * 256];
    std::memcpy(buffer, "BESP", 4);
    std::memcpy(buffer + 4, values, 4 * count);
    sendto(m_socket, buffer, 4 + 4 * count, 0, reinterpret_cast<const sockaddr *>(&to), sizeof(to));
}

void BridgeTransport::sendAudio(const int16_t *samples, int count, float gain) {
    if (m_socket < 0) return;

    std::lock_guard<std::mutex> lock(m_audioMutex);
    for (int i = 0; i < count; ++i) m_audioQueue.push_back(samples[i] * (gain / 32768.0f));

    // Never hold more than a second; the plugin resyncs if it falls this far behind
    while (m_audioQueue.size() > 44100) m_audioQueue.pop_front();
}

void BridgeTransport::audioThread() {
    const sockaddr_in plugin = localhost(m_audioPort);

    // Paces audio out at 44.1 kHz in small packets, nudging the rate to keep ~34 ms queued
    constexpr int QueueTarget = 1500;
    constexpr int MaxPacket = 600;
    char buffer[4 + 4 * MaxPacket];
    std::memcpy(buffer, "BESF", 4);

    auto last = std::chrono::steady_clock::now();
    double due = 0;
    while (m_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        const auto now = std::chrono::steady_clock::now();
        due += std::chrono::duration<double>(now - last).count() * 44100.0;
        last = now;

        int n;
        {
            std::lock_guard<std::mutex> lock(m_audioMutex);
            const int queued = static_cast<int>(m_audioQueue.size());
            due += (queued > 2 * QueueTarget) ? (queued - 2 * QueueTarget) * 0.05 : 0;
            if (queued < QueueTarget / 2 && due > queued) due = 0;

            n = static_cast<int>(std::min(due, static_cast<double>(std::min(queued, MaxPacket))));
            for (int i = 0; i < n; ++i) {
                std::memcpy(buffer + 4 + 4 * i, &m_audioQueue.front(), 4);
                m_audioQueue.pop_front();
            }
        }

        due -= n;
        if (due > 4410) due = 4410;
        if (n > 0) {
            sendto(m_socket, buffer, 4 + 4 * n, 0, reinterpret_cast<const sockaddr *>(&plugin), sizeof(plugin));
        }
    }
}
