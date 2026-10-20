// engine-sim-headless: the simulation without any graphics or audio device.
// Audio goes to listeners (a WAV file for now; the BeamNG bridge later).

#include "control_server.h"
#include "engine_host.h"
#include "host_settings.h"

#include "build_info.h"
#include "engine.h"
#include "ignition_module.h"
#include "simulator.h"
#include "units.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

namespace {
    std::atomic<bool> g_quit{false};

    void onSignal(int) { g_quit = true; }

    struct Options {
        std::string script;
        std::string wavPath;
        double throttle = 0.0;
        double duration = 0.0;
        bool start = false;
        bool quiet = false;
        bool bridge = true;
    };

    void printUsage(const char *argv0) {
        std::printf(
            "Usage: %s [options]\n"
            "\n"
            "Runs the engine simulation without graphics.\n"
            "\n"
            "  --script PATH     Engine script to load this time (default: the saved one, else main.mr)\n"
            "  --start           Turn on ignition and crank until the engine runs\n"
            "  --throttle X      Hold the throttle at X (0 to 1)\n"
            "  --duration SEC    Exit after SEC seconds (default: run until Ctrl+C)\n"
            "  --wav PATH        Record the engine audio to a WAV file\n"
            "  --no-bridge       Don't listen for BeamNG on 127.0.0.1:4570\n"
            "  --quiet           Don't print status once a second\n"
            "  --help            Show this help\n",
            argv0);
    }

    bool parseOptions(int argc, char **argv, Options *options) {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto value = [&](const char *name) -> const char * {
                if (i + 1 >= argc) {
                    std::fprintf(stderr, "%s needs a value\n", name);
                    return nullptr;
                }
                return argv[++i];
            };

            if (arg == "--help" || arg == "-h") { printUsage(argv[0]); std::exit(0); }
            else if (arg == "--start") options->start = true;
            else if (arg == "--quiet") options->quiet = true;
            else if (arg == "--no-bridge") options->bridge = false;
            else if (arg == "--script") { const char *v = value("--script"); if (!v) return false; options->script = v; }
            else if (arg == "--wav") { const char *v = value("--wav"); if (!v) return false; options->wavPath = v; }
            else if (arg == "--throttle") { const char *v = value("--throttle"); if (!v) return false; options->throttle = std::atof(v); }
            else if (arg == "--duration") { const char *v = value("--duration"); if (!v) return false; options->duration = std::atof(v); }
            else {
                std::fprintf(stderr, "Unknown option: %s\n", arg.c_str());
                return false;
            }
        }

        if (options->throttle < 0 || options->throttle > 1) {
            std::fprintf(stderr, "--throttle must be between 0 and 1\n");
            return false;
        }

        return true;
    }

    // 16-bit mono PCM; sizes are patched in when the file is closed
    class WavWriter {
    public:
        bool open(const std::string &path, int sampleRate) {
            m_file.open(path, std::ios::binary);
            if (!m_file) return false;

            const uint32_t byteRate = sampleRate * 2;
            writeTag("RIFF"); write32(0); writeTag("WAVE");
            writeTag("fmt "); write32(16); write16(1); write16(1);
            write32(sampleRate); write32(byteRate); write16(2); write16(16);
            writeTag("data"); write32(0);
            return true;
        }

        void write(const int16_t *samples, int count) {
            m_file.write(reinterpret_cast<const char *>(samples), count * sizeof(int16_t));
            m_samples += count;
        }

        void close() {
            if (!m_file.is_open()) return;
            const uint32_t dataBytes = static_cast<uint32_t>(m_samples * 2);
            m_file.seekp(4); write32(36 + dataBytes);
            m_file.seekp(40); write32(dataBytes);
            m_file.close();
        }

        long long samples() const { return m_samples; }

    private:
        void writeTag(const char *tag) { m_file.write(tag, 4); }
        void write16(uint16_t v) { const char b[2] = { char(v), char(v >> 8) }; m_file.write(b, 2); }
        void write32(uint32_t v) { const char b[4] = { char(v), char(v >> 8), char(v >> 16), char(v >> 24) }; m_file.write(b, 4); }

        std::ofstream m_file;
        long long m_samples = 0;
    };
}

int main(int argc, char **argv) {
    Options options;
    if (!parseOptions(argc, argv, &options)) {
        printUsage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    const HostPaths paths = HostPaths::resolve();
    const std::string settingsFile = paths.configPath + "/settings.json";

    std::string settingsError;
    const HostSettings settings = HostSettings::load(settingsFile, &settingsError);
    if (!settingsError.empty()) std::cerr << "engine-sim: " << settingsError << std::endl;

    EngineHost host(paths);

    const EngineHost::LoadResult load = host.loadScript(options.script.empty() ? settings.script : options.script);
    if (!load.errors.empty()) {
        std::cerr << "engine-sim: " << load.script << " failed to load:\n" << load.errors << std::flush;
    }
    if (!load.engineLoaded) {
        std::cerr << "engine-sim: " << load.message << std::endl;
        return 1;
    }
    std::cerr << "engine-sim " ENGINE_SIM_PROJECT_VERSION ": " << load.message << std::endl;

    if (options.bridge) {
        if (host.enableBridge()) {
            std::cerr << "engine-sim: waiting for BeamNG on 127.0.0.1:" << BridgeTransport::portFromEnvironment() << std::endl;
        }
        else {
            std::cerr << "engine-sim: could not listen on port " << BridgeTransport::portFromEnvironment()
                      << " (another engine-sim running?); BeamNG bridge disabled" << std::endl;
        }
    }

    applySettings(&host, settings);

    ControlServer control(&host, settings, settingsFile);
    control.setShutdownHandler([] { g_quit = true; });
    std::string controlError;
    if (control.start(ControlServer::defaultSocketPath(), &controlError)) {
        std::cerr << "engine-sim: control socket at " << control.socketPath() << std::endl;
    }
    else {
        std::cerr << "engine-sim: " << controlError << "; control socket disabled" << std::endl;
    }

    WavWriter wav;
    if (!options.wavPath.empty() && !wav.open(options.wavPath, EngineHost::AudioSampleRate)) {
        std::cerr << "engine-sim: could not write " << options.wavPath << std::endl;
        return 1;
    }

    // Filled on the simulation thread, read once a second for the status line
    std::atomic<long long> iterations{0};
    std::atomic<double> busySeconds{0};

    host.setAudioHook([&](const int16_t *samples, int count) {
        if (!options.wavPath.empty()) wav.write(samples, count);
    });
    host.setFrameHook([&](const EngineHost::FrameStats &stats) {
        iterations += stats.iterations;
        busySeconds = busySeconds + stats.processingSeconds;
    });

    double starterSpeedRpm = 0;
    {
        auto lock = host.lock();
        Engine *engine = host.engine();
        engine->setSpeedControl(options.throttle);
        starterSpeedRpm = units::toRpm(engine->getStarterSpeed());
        if (options.start) {
            engine->getIgnitionModule()->m_enabled = true;
            host.simulator()->m_starterMotor.m_enabled = true;
        }
    }

    host.start();

    using clock = std::chrono::steady_clock;
    const auto startTime = clock::now();
    auto lastStatus = startTime;
    bool cranking = options.start;

    while (!g_quit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        host.runPending();

        const auto now = clock::now();
        const double elapsed = std::chrono::duration<double>(now - startTime).count();
        if (options.duration > 0 && elapsed >= options.duration) break;

        double rpm;
        std::string bridgeMessage;
        {
            auto lock = host.lock();
            rpm = host.engine()->getRpm();
            if (host.bridge() != nullptr) bridgeMessage = host.bridge()->takeLogMessage();

            // BeamNG drives the engine while linked
            if (host.bridgeActive()) cranking = false;

            // Let go of the starter once the engine is clearly running on its own
            if (cranking && (rpm > 2.5 * starterSpeedRpm || elapsed > 5.0)) {
                host.simulator()->m_starterMotor.m_enabled = false;
                cranking = false;
            }
        }

        if (!bridgeMessage.empty()) std::fprintf(stderr, "%6.1fs  %s\n", elapsed, bridgeMessage.c_str());

        const double sinceStatus = std::chrono::duration<double>(now - lastStatus).count();
        if (!options.quiet && sinceStatus >= 1.0) {
            const double load = busySeconds.exchange(0) / sinceStatus;
            const long long steps = iterations.exchange(0);
            std::fprintf(stderr, "%6.1fs  %6.0f rpm  %7.0f steps/s  load %3.0f%%%s%s\n",
                elapsed, rpm, steps / sinceStatus, load * 100,
                cranking ? "  cranking" : "",
                host.bridgeActive() ? "  [BeamNG]" : "");
            lastStatus = now;
        }
    }

    control.stop();
    host.stop();
    wav.close();

    if (!options.wavPath.empty()) {
        std::cerr << "engine-sim: wrote " << wav.samples() / double(EngineHost::AudioSampleRate)
                  << " s of audio to " << options.wavPath << std::endl;
    }

    return 0;
}
