#include "launcher.h"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;

extern char **environ;

namespace {
    const char *binaryName(EngineLauncher::Mode mode) {
        return mode == EngineLauncher::Mode::Headless ? "engine-sim-headless" : "engine-sim";
    }

    // Next to the panel when installed; in the sibling build folders when run from a build;
    // otherwise left to PATH
    std::string findBinary(EngineLauncher::Mode mode) {
        std::error_code ec;
        const fs::path self = fs::read_symlink("/proc/self/exe", ec);
        if (!ec) {
            const fs::path dir = self.parent_path();
            const char *name = binaryName(mode);
            const fs::path candidates[] = {
                dir / name,
                dir / ".." / (mode == EngineLauncher::Mode::Headless ? "headless" : "gui") / name,
            };
            for (const fs::path &candidate : candidates) {
                if (access(candidate.c_str(), X_OK) == 0) return fs::weakly_canonical(candidate).string();
            }
        }
        return binaryName(mode);
    }

    std::string lastLines(const std::string &path, int count) {
        std::ifstream file(path);
        std::vector<std::string> lines;
        for (std::string line; std::getline(file, line);) lines.push_back(line);

        std::string tail;
        for (size_t i = lines.size() > size_t(count) ? lines.size() - count : 0; i < lines.size(); ++i) {
            tail += lines[i] + "\n";
        }
        return tail;
    }
}

EngineLauncher::~EngineLauncher() {
    // stop() is the polite path; this only makes sure nothing is left behind
    if (m_pid > 0 && running()) kill(m_pid, SIGTERM);
}

std::string EngineLauncher::logPath() {
    const char *state = std::getenv("XDG_STATE_HOME");
    const char *home = std::getenv("HOME");
    const fs::path dir = (state != nullptr && state[0] != '\0')
        ? fs::path(state) / "engine-sim"
        : fs::path(home != nullptr ? home : "/tmp") / ".local/state/engine-sim";

    std::error_code ec;
    fs::create_directories(dir, ec);
    return (dir / "engine-sim.log").string();
}

bool EngineLauncher::start(Mode mode, std::string *error) {
    if (m_pid > 0 && running()) return true;

    const std::string binary = findBinary(mode);
    const std::string log = logPath();

    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);

    // Its own process group, so Ctrl+C in the panel's terminal doesn't skip the clean stop
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
    posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
    posix_spawnattr_setpgroup(&attributes, 0);

    std::vector<char *> argv = { const_cast<char *>(binary.c_str()) };
    if (mode == Mode::Headless) argv.push_back(const_cast<char *>("--quiet"));
    argv.push_back(nullptr);

    pid_t pid = -1;
    const int result = posix_spawnp(&pid, binary.c_str(), &actions, &attributes, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    posix_spawnattr_destroy(&attributes);

    if (result != 0) {
        *error = "couldn't start " + binary + ": " + std::strerror(result);
        return false;
    }

    m_pid = pid;
    m_exitMessage.clear();
    return true;
}

bool EngineLauncher::running() {
    if (m_pid <= 0) return false;

    int status = 0;
    const pid_t done = waitpid(m_pid, &status, WNOHANG);
    if (done == 0) return true;

    if (done == m_pid) {
        if (WIFSIGNALED(status)) {
            m_exitMessage = std::string("engine-sim crashed (") + strsignal(WTERMSIG(status)) + ")";
        }
        else if (WEXITSTATUS(status) != 0) {
            m_exitMessage = "engine-sim exited with code " + std::to_string(WEXITSTATUS(status));
        }
        if (!m_exitMessage.empty()) m_exitMessage += "\n" + lastLines(logPath(), 8);
    }

    m_pid = -1;
    return false;
}

void EngineLauncher::stop(const std::string &socketPath) {
    if (m_pid <= 0 || !running()) return;

    requestShutdown(socketPath);
    for (int i = 0; i < 50 && running(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (running()) {
        kill(m_pid, SIGTERM);
        for (int i = 0; i < 30 && running(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (running()) {
        kill(m_pid, SIGKILL);
        waitpid(m_pid, nullptr, 0);
        m_pid = -1;
    }
}

bool requestShutdown(const std::string &socketPath) {
    sockaddr_un address{};
    if (socketPath.size() >= sizeof(address.sun_path)) return false;
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, socketPath.c_str(), socketPath.size() + 1);

    const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return false;

    bool ok = false;
    if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0) {
        const char request[] = "{\"cmd\":\"shutdown\"}\n";
        ok = send(fd, request, sizeof(request) - 1, MSG_NOSIGNAL) == ssize_t(sizeof(request) - 1);

        // Wait briefly for the reply so the request isn't cut off
        char reply[256];
        timeval timeout{ 1, 0 };
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        recv(fd, reply, sizeof(reply), 0);
    }

    close(fd);
    return ok;
}
