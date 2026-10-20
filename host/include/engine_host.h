#ifndef ATG_ENGINE_SIM_HOST_ENGINE_HOST_H
#define ATG_ENGINE_SIM_HOST_ENGINE_HOST_H

#include "application_settings.h"
#include "beam_bridge.h"
#include "bridge_transport.h"

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class Engine;
class Simulator;
class Transmission;
class Vehicle;

// Where engine-sim reads and writes files
struct HostPaths {
    std::string dataRoot;   // Shipped data: assets/, es/, delta/
    std::string userData;   // User overrides, e.g. assets/main.mr (~/.local/share/engine-sim)
    std::string outputPath; // Logs (~/.local/state/engine-sim)
    std::string configPath; // Settings (~/.config/engine-sim)

    // ENGINE_SIM_DATA_ROOT, then the installed data, then the source tree; XDG dirs for the rest
    static HostPaths resolve();
};

// Owns the loaded engine and runs the simulation on its own thread, independent of any UI.
//
// Threading: the simulation thread holds mutex() while it steps. Anything touching the
// simulation from another thread (reading state for display, changing controls) must hold
// it too, via lock(). Hooks run on the simulation thread with the lock already held.
class EngineHost {
public:
    static constexpr int AudioSampleRate = 44100;

    struct LoadResult {
        bool engineLoaded = false;  // An engine is running (new, fallback or the previous one)
        bool replaced = false;      // The running engine changed
        bool usedFallback = false;  // The requested script failed and the default was loaded
        std::string script;         // Script that was requested
        std::string message;        // One line summary, suitable for display
        std::string errors;         // Compiler output when the script failed
    };

    struct FrameStats {
        int iterations = 0;              // Simulation steps this tick
        double processingSeconds = 0;    // Wall time spent stepping
        double inputLatency = 0;         // Synthesizer input buffer, seconds
        double inputLatencyTarget = 0;
    };

    using StepHook = std::function<void()>;
    using AudioHook = std::function<void(const int16_t *samples, int count)>;
    using FrameHook = std::function<void(const FrameStats &stats)>;
    using EngineChangedHook = std::function<void()>;

    explicit EngineHost(HostPaths paths);
    ~EngineHost();

    EngineHost(const EngineHost &) = delete;
    EngineHost &operator=(const EngineHost &) = delete;

    const HostPaths &paths() const { return m_paths; }

    // The user's main.mr if it exists, otherwise the shipped one
    std::string defaultScriptPath() const;

    // Compiles a script and swaps its engine in. Either a main.mr-style script, or an engine
    // file defining `main` (it gets wrapped in a generated main.mr). Compilation runs without the lock, so the
    // current engine keeps running meanwhile; only the swap takes it. If the script fails, the
    // current engine is kept, or with none loaded yet the shipped default is tried.
    // Must not be called while holding lock().
    LoadResult loadScript(const std::string &scriptPath = "");

    // Listens for BeamNG on 127.0.0.1:port. Call before start(); false if the port is taken.
    bool enableBridge(int port = BridgeTransport::portFromEnvironment(), int audioPort = BridgeTransport::audioPortFromEnvironment());

    // Following BeamNG right now (safe without the lock)
    bool bridgeActive() const { return m_bridge != nullptr && m_bridge->active(); }

    // Simulation thread
    void start();
    void stop();
    bool running() const { return m_running; }

    // Work that must happen on the thread that owns the host (the GUI's main thread, or
    // headless's main loop), such as loading engines requested over the control socket.
    // post() is safe from any thread; the owner calls runPending() regularly, without the lock.
    void post(std::function<void()> job);
    void runPending();

    // Measured over the last second (safe without the lock)
    double stepsPerSecond() const { return m_stepsPerSecond; }
    double load() const { return m_load; } // Fraction of real time spent simulating

    std::unique_lock<std::mutex> lock() { return std::unique_lock<std::mutex>(m_mutex); }

    // --- Everything below requires lock() ---

    Simulator *simulator() { return m_simulator; }
    Engine *engine() { return m_engine; }
    Vehicle *vehicle() { return m_vehicle; }
    Transmission *transmission() { return m_transmission; }
    const ApplicationSettings &applicationSettings() const { return m_applicationSettings; }
    const std::string &loadedScript() const { return m_loadedScript; }
    BeamBridge *bridge() { return m_bridge.get(); } // nullptr unless enableBridge() succeeded

    // 0 uses each engine's own rate. Applies now and to engines loaded later.
    void setSimulationFrequencyOverride(double hz);

    // Lowers the synthesizer's level target by this many dB so its peaks stop clipping.
    // Applies now and to engines loaded later.
    void setOutputHeadroom(double db);
    double outputHeadroom() const { return m_outputHeadroomDb; }
    double simulationFrequencyOverride() const { return m_frequencyOverride; }

    void setSimulationSpeed(double speed) { m_simulationSpeed = speed; }
    double simulationSpeed() const { return m_simulationSpeed; }
    void setPaused(bool paused) { m_paused = paused; }
    bool paused() const { return m_paused; }
    void stepOnce() { m_stepOnce = true; }

    // Hooks. Set them before start(); they run with the lock held.
    void setStepHook(StepHook hook) { m_stepHook = std::move(hook); }             // After every simulation step
    void setAudioHook(AudioHook hook) { m_audioHook = std::move(hook); }          // Audio, at real-time pace
    void setFrameHook(FrameHook hook) { m_frameHook = std::move(hook); }          // After each tick
    void setEngineChangedHook(EngineChangedHook hook) { m_engineChangedHook = std::move(hook); } // On the loadScript() caller's thread

private:
    struct Compiled {
        Engine *engine = nullptr;
        Vehicle *vehicle = nullptr;
        Transmission *transmission = nullptr;
        ApplicationSettings settings;
    };

    // `noEngine` is set when the script compiled fine but never created an engine (an engine
    // file rather than a main.mr), so the caller can retry it wrapped
    bool compile(const std::string &scriptPath, const std::string &logPath, Compiled *out, std::string *errors, bool *noEngine = nullptr);
    std::string writeWrapper(const std::string &engineFile);
    bool prepare(Compiled *compiled, Simulator **simulator, std::string *errors);
    void install(const Compiled &compiled, Simulator *simulator, const std::string &script);
    static void release(Engine *engine, Vehicle *vehicle, Transmission *transmission, Simulator *simulator);

    void simulationThread();
    void tick(double dt);

private:
    HostPaths m_paths;

    std::mutex m_mutex;
    std::thread m_thread;
    std::atomic<bool> m_running{false};

    Engine *m_engine = nullptr;
    Vehicle *m_vehicle = nullptr;
    Transmission *m_transmission = nullptr;
    Simulator *m_simulator = nullptr;
    ApplicationSettings m_applicationSettings;
    std::string m_loadedScript;

    double m_simulationSpeed = 1.0;
    double m_frequencyOverride = 0;
    double m_outputHeadroomDb = 6;
    bool m_paused = false;
    bool m_stepOnce = false;

    std::unique_ptr<BridgeTransport> m_transport;
    std::unique_ptr<BeamBridge> m_bridge;

    double m_audioDue = 0;

    std::mutex m_jobMutex;
    std::deque<std::function<void()>> m_jobs;

    std::atomic<double> m_stepsPerSecond{0};
    std::atomic<double> m_load{0};
    double m_statWindow = 0;
    long long m_statSteps = 0;
    double m_statBusy = 0;
    std::vector<int16_t> m_audioScratch;

    StepHook m_stepHook;
    AudioHook m_audioHook;
    FrameHook m_frameHook;
    EngineChangedHook m_engineChangedHook;
};

#endif /* ATG_ENGINE_SIM_HOST_ENGINE_HOST_H */
