#ifndef ENGINE_SIM_CONTROL_LAUNCHER_H
#define ENGINE_SIM_CONTROL_LAUNCHER_H

#include <string>
#include <sys/types.h>

// Starts and stops an engine-sim process for the control panel
class EngineLauncher {
public:
    enum class Mode { Headless, Window };

    ~EngineLauncher();

    // Output goes to ~/.local/state/engine-sim/engine-sim.log
    bool start(Mode mode, std::string *error);

    // Whether the process we started is still alive. When it has exited, `exitMessage`
    // explains how, with the end of its log.
    bool running();
    const std::string &exitMessage() const { return m_exitMessage; }
    bool launched() const { return m_pid > 0; }

    // Asks it to exit over the control socket, then signals if it doesn't
    void stop(const std::string &socketPath);

    static std::string logPath();

private:
    pid_t m_pid = -1;
    std::string m_exitMessage;
};

// Asks whatever engine-sim is listening on the socket to exit
bool requestShutdown(const std::string &socketPath);

#endif /* ENGINE_SIM_CONTROL_LAUNCHER_H */
