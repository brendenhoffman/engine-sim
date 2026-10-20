#include "engine_host.h"

#include "build_info.h"
#include "compiler.h"
#include "engine.h"
#include "exhaust_system.h"
#include "impulse_response.h"
#include "simulator.h"
#include "transmission.h"
#include "units.h"
#include "vehicle.h"
#include "wav_reader.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace {
    // $XDG_VAR/engine-sim, falling back to ~/<fallback>/engine-sim; created if missing
    std::string xdgDir(const char *variable, const char *fallback) {
        const char *base = std::getenv(variable);
        const char *home = std::getenv("HOME");
        fs::path path = (base != nullptr && base[0] != '\0')
            ? fs::path(base)
            : fs::path(home != nullptr ? home : ".") / fallback;
        path /= "engine-sim";

        std::error_code ec;
        fs::create_directories(path, ec);
        return path.string();
    }

    std::string readFile(const std::string &path) {
        std::ifstream file(path);
        std::stringstream contents;
        contents << file.rdbuf();
        return contents.str();
    }
}

HostPaths HostPaths::resolve() {
    HostPaths paths;

    const char *dataOverride = std::getenv("ENGINE_SIM_DATA_ROOT");
    if (dataOverride != nullptr && dataOverride[0] != '\0') {
        paths.dataRoot = dataOverride;
    }
    else if (fs::exists(ENGINE_SIM_DATA_ROOT)) {
        paths.dataRoot = ENGINE_SIM_DATA_ROOT;
    }
    else {
        // Running from a build directory without installing
        paths.dataRoot = ENGINE_SIM_SOURCE_DATA_ROOT;
    }

    paths.userData = xdgDir("XDG_DATA_HOME", ".local/share");
    paths.outputPath = xdgDir("XDG_STATE_HOME", ".local/state");

    return paths;
}

EngineHost::EngineHost(HostPaths paths) : m_paths(std::move(paths)) {
    /* void */
}

EngineHost::~EngineHost() {
    stop();

    std::lock_guard<std::mutex> lock(m_mutex);
    release(m_engine, m_vehicle, m_transmission, m_simulator);
    m_engine = nullptr;
    m_vehicle = nullptr;
    m_transmission = nullptr;
    m_simulator = nullptr;
}

std::string EngineHost::defaultScriptPath() const {
    const fs::path userScript = fs::path(m_paths.userData) / "assets" / "main.mr";
    if (fs::exists(userScript)) return userScript.string();

    return (fs::path(m_paths.dataRoot) / "assets" / "main.mr").string();
}

bool EngineHost::compile(
    const std::string &scriptPath,
    const std::string &logPath,
    Compiled *out,
    std::string *errors)
{
    // User assets take priority over shipped ones
    std::vector<piranha::IrPath> searchPaths;
    searchPaths.push_back((fs::path(m_paths.userData) / "assets").string());
    searchPaths.push_back((fs::path(m_paths.dataRoot) / "assets").string());
    searchPaths.push_back((fs::path(m_paths.dataRoot) / "es").string());

    bool loaded = false;
    {
        std::ofstream log(logPath, std::ios::out);

        es_script::Compiler compiler;
        compiler.initialize(searchPaths);
        if (compiler.compile(scriptPath, log)) {
            const es_script::Compiler::Output output = compiler.execute();

            // A script can compile without ever calling main()/set_engine()
            loaded = (output.engine != nullptr);
            if (loaded) {
                out->engine = output.engine;
                out->vehicle = output.vehicle;
                out->transmission = output.transmission;
                out->settings = output.applicationSettings;
            }
            else {
                log << scriptPath << ": script compiled but did not create an engine" << std::endl;
            }
        }

        compiler.destroy();
    }

    if (!loaded) {
        *errors = readFile(logPath);
        return false;
    }

    // Scripts may leave these out; use the same stand-ins upstream engine-sim does
    if (out->vehicle == nullptr) {
        Vehicle::Parameters vehParams;
        vehParams.mass = units::mass(1597, units::kg);
        vehParams.diffRatio = 3.42;
        vehParams.tireRadius = units::distance(10, units::inch);
        vehParams.dragCoefficient = 0.25;
        vehParams.crossSectionArea = units::distance(6.0, units::foot) * units::distance(6.0, units::foot);
        vehParams.rollingResistance = 2000.0;
        out->vehicle = new Vehicle;
        out->vehicle->initialize(vehParams);
    }

    if (out->transmission == nullptr) {
        const double gearRatios[] = { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };
        Transmission::Parameters tParams;
        tParams.GearCount = 6;
        tParams.GearRatios = gearRatios;
        tParams.MaxClutchTorque = units::torque(1000.0, units::ft_lb);
        out->transmission = new Transmission;
        out->transmission->initialize(tParams);
    }

    return true;
}

bool EngineHost::prepare(Compiled *compiled, Simulator **simulatorOut, std::string *errors) {
    Engine *engine = compiled->engine;

    // Load impulse responses first so a missing file fails before anything is built
    std::vector<std::vector<int16_t>> impulses(engine->getExhaustSystemCount());
    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        ImpulseResponse *response = engine->getExhaustSystem(i)->getImpulseResponse();
        std::string error;
        if (!loadWavMono16(response->getFilename(), AudioSampleRate, &impulses[i], &error)) {
            *errors = "Could not load impulse response: " + error;
            return false;
        }
    }

    engine->calculateDisplacement();

    Simulator *simulator = engine->createSimulator(compiled->vehicle, compiled->transmission);
    simulator->setSimulationFrequency(engine->getSimulationFrequency());

    Synthesizer::AudioParameters audioParams = simulator->synthesizer().getAudioParameters();
    audioParams.inputSampleNoise = static_cast<float>(engine->getInitialJitter());
    audioParams.airNoise = static_cast<float>(engine->getInitialNoise());
    audioParams.dF_F_mix = static_cast<float>(engine->getInitialHighFrequencyGain());
    simulator->synthesizer().setAudioParameters(audioParams);

    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        ImpulseResponse *response = engine->getExhaustSystem(i)->getImpulseResponse();
        simulator->synthesizer().initializeImpulseResponse(
            impulses[i].data(),
            static_cast<unsigned int>(impulses[i].size()),
            static_cast<float>(response->getVolume()),
            i);
    }

    simulator->startAudioRenderingThread();

    *simulatorOut = simulator;
    return true;
}

void EngineHost::install(const Compiled &compiled, Simulator *simulator) {
    std::lock_guard<std::mutex> lock(m_mutex);

    Engine *oldEngine = m_engine;
    Vehicle *oldVehicle = m_vehicle;
    Transmission *oldTransmission = m_transmission;
    Simulator *oldSimulator = m_simulator;

    m_engine = compiled.engine;
    m_vehicle = compiled.vehicle;
    m_transmission = compiled.transmission;
    m_simulator = simulator;
    m_applicationSettings = compiled.settings;
    m_audioDue = 0;

    // Consumers drop anything pointing into the old engine before it's freed
    if (m_engineChangedHook) m_engineChangedHook();

    release(oldEngine, oldVehicle, oldTransmission, oldSimulator);
}

void EngineHost::release(Engine *engine, Vehicle *vehicle, Transmission *transmission, Simulator *simulator) {
    // The simulator references the rest, so it goes first
    if (simulator != nullptr) {
        simulator->releaseSimulation();
        delete simulator;
    }

    delete vehicle;
    delete transmission;

    if (engine != nullptr) {
        engine->destroy();
        delete engine;
    }
}

EngineHost::LoadResult EngineHost::loadScript(const std::string &requestedPath) {
    LoadResult result;
    result.script = requestedPath.empty() ? defaultScriptPath() : requestedPath;

    const std::string logPath = (fs::path(m_paths.outputPath) / "error_log.log").string();
    const std::string shippedScript = (fs::path(m_paths.dataRoot) / "assets" / "main.mr").string();

    Compiled compiled;
    Simulator *simulator = nullptr;
    std::string errors;

    auto discard = [](Compiled &c) {
        delete c.vehicle;
        delete c.transmission;
        if (c.engine != nullptr) {
            c.engine->destroy();
            delete c.engine;
        }
        c = Compiled();
    };

    bool ok = compile(result.script, logPath, &compiled, &errors);
    if (ok && !prepare(&compiled, &simulator, &errors)) {
        std::ofstream(logPath, std::ios::app) << errors << std::endl;
        discard(compiled);
        ok = false;
    }

    if (!ok) {
        result.errors = errors;

        bool haveEngine;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            haveEngine = (m_engine != nullptr);
        }

        if (haveEngine) {
            result.engineLoaded = true;
            result.message = "Script failed, kept the current engine. See " + logPath;
            return result;
        }

        // Nothing running yet: fall back to the shipped default, keeping the user's error log intact
        std::string fallbackErrors;
        const std::string fallbackLog = (fs::path(m_paths.outputPath) / "default_error_log.log").string();
        const bool fallbackOk =
            result.script != shippedScript
            && compile(shippedScript, fallbackLog, &compiled, &fallbackErrors)
            && prepare(&compiled, &simulator, &fallbackErrors);

        if (!fallbackOk) {
            discard(compiled);
            result.message = "Script failed and so did the default engine. See " + logPath;
            return result;
        }

        result.usedFallback = true;
        result.message = "Script failed, loaded the default engine. See " + logPath;
    }

    install(compiled, simulator);

    result.engineLoaded = true;
    result.replaced = true;
    if (result.message.empty()) {
        result.message = "Loaded " + compiled.engine->getName();
    }

    return result;
}

void EngineHost::start() {
    if (m_running) return;

    m_running = true;
    m_thread = std::thread(&EngineHost::simulationThread, this);
}

void EngineHost::stop() {
    if (!m_running) return;

    m_running = false;
    if (m_thread.joinable()) m_thread.join();
}

void EngineHost::simulationThread() {
    using clock = std::chrono::steady_clock;

    auto last = clock::now();
    while (m_running) {
        // Small, frequent ticks keep each hold of the lock short
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        const auto now = clock::now();
        const double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        std::lock_guard<std::mutex> lock(m_mutex);
        tick(dt);
    }
}

void EngineHost::tick(double dt) {
    if (m_simulator == nullptr || m_engine == nullptr) return;

    if (m_paused && !m_stepOnce) {
        m_audioDue = 0;
        return;
    }
    m_stepOnce = false;

    // If the thread was held up, don't try to catch up more than a frame's worth at once
    dt = std::min(dt, 1 / 30.0);

    m_simulator->setSimulationSpeed(m_simulationSpeed);
    m_simulator->startFrame(dt);

    FrameStats stats;
    stats.iterations = m_simulator->getFrameIterationCount();

    const auto t0 = std::chrono::steady_clock::now();
    while (m_simulator->simulateStep()) {
        if (m_stepHook) m_stepHook();
    }
    const auto t1 = std::chrono::steady_clock::now();

    m_simulator->endFrame();

    stats.processingSeconds = std::chrono::duration<double>(t1 - t0).count();
    stats.inputLatency = m_simulator->getSynthesizerInputLatency();
    stats.inputLatencyTarget = m_simulator->getSynthesizerInputLatencyTarget();
    if (m_frameHook) m_frameHook(stats);

    // Drain audio at the wall-clock rate. The synthesizer only renders while its output has
    // room, so this read rate is what holds the simulation to real time.
    constexpr int MaxChunk = 4096;
    m_audioScratch.resize(MaxChunk);
    m_audioDue = std::min(m_audioDue + dt * AudioSampleRate, 0.1 * AudioSampleRate);
    while (m_audioDue >= 1) {
        const int wanted = std::min(static_cast<int>(m_audioDue), MaxChunk);
        const int read = m_simulator->readAudioOutput(wanted, m_audioScratch.data());
        if (read <= 0) break;

        m_audioDue -= read;
        if (m_audioHook) m_audioHook(m_audioScratch.data(), read);

        if (read < wanted) break;
    }

    // Starved: don't build up a debt that would later be read as a burst
    m_audioDue = std::min(m_audioDue, 0.02 * AudioSampleRate);
}
