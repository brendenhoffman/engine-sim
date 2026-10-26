#include "control_server.h"
#include "forced_induction.h"
#include "intake.h"

#include "engine.h"
#include "engine_generator.h"
#include "engine_host.h"
#include "ignition_module.h"
#include "simulator.h"
#include "units.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <future>
#include <iostream>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;
using nlohmann::json;

namespace {
    json failure(const std::string &message) {
        return { { "ok", false }, { "error", message } };
    }

    bool fillAddress(const std::string &path, sockaddr_un *address) {
        if (path.size() >= sizeof(address->sun_path)) return false;

        std::memset(address, 0, sizeof(*address));
        address->sun_family = AF_UNIX;
        std::memcpy(address->sun_path, path.c_str(), path.size() + 1);
        return true;
    }

    // Whether something is accepting connections on the socket path
    bool socketInUse(const std::string &path) {
        sockaddr_un address;
        if (!fillAddress(path, &address)) return false;

        const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if (fd < 0) return false;

        const bool inUse = connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0;
        close(fd);
        return inUse;
    }

    void sendLine(int fd, const json &reply) {
        const std::string line = reply.dump() + "\n";
        size_t sent = 0;
        while (sent < line.size()) {
            const ssize_t n = send(fd, line.data() + sent, line.size() - sent, MSG_NOSIGNAL);
            if (n <= 0) return; // Client went away; poll() will notice
            sent += static_cast<size_t>(n);
        }
    }
}

ControlServer::ControlServer(EngineHost *host, HostSettings settings, std::string settingsFile)
    : m_host(host), m_settingsFile(std::move(settingsFile)), m_settings(std::move(settings))
{
    /* void */
}

ControlServer::~ControlServer() {
    stop();
}

std::string ControlServer::defaultSocketPath() {
    const char *runtime = std::getenv("XDG_RUNTIME_DIR");
    if (runtime != nullptr && runtime[0] != '\0') {
        return (fs::path(runtime) / "engine-sim.sock").string();
    }

    return "/tmp/engine-sim-" + std::to_string(getuid()) + ".sock";
}

bool ControlServer::start(const std::string &socketPath, std::string *error) {
    if (m_running) return true;

    sockaddr_un address;
    if (!fillAddress(socketPath, &address)) {
        *error = "socket path too long: " + socketPath;
        return false;
    }

    if (fs::exists(socketPath)) {
        if (socketInUse(socketPath)) {
            *error = "another engine-sim is already running (" + socketPath + ")";
            return false;
        }
        unlink(socketPath.c_str()); // Left behind by a crash
    }

    const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0 || bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 || listen(fd, 8) != 0) {
        *error = "could not listen on " + socketPath + ": " + std::strerror(errno);
        if (fd >= 0) close(fd);
        return false;
    }

    m_listenSocket = fd;
    m_socketPath = socketPath;
    m_running = true;
    m_thread = std::thread(&ControlServer::serverThread, this);
    return true;
}

void ControlServer::stop() {
    if (!m_running) return;

    m_running = false;
    if (m_thread.joinable()) m_thread.join();

    close(m_listenSocket);
    m_listenSocket = -1;
    unlink(m_socketPath.c_str());
}

void ControlServer::serverThread() {
    struct Client {
        int fd;
        std::string buffer;
    };
    std::vector<Client> clients;

    while (m_running) {
        std::vector<pollfd> waitFor;
        waitFor.push_back({ m_listenSocket, POLLIN, 0 });
        for (const Client &client : clients) waitFor.push_back({ client.fd, POLLIN, 0 });

        maybeAutoGenerate();

        // Wake up regularly so stop() doesn't hang
        if (poll(waitFor.data(), waitFor.size(), 100) <= 0) continue;

        if (waitFor[0].revents & POLLIN) {
            const int fd = accept4(m_listenSocket, nullptr, nullptr, SOCK_CLOEXEC);
            if (fd >= 0) clients.push_back({ fd, "" });
        }

        for (size_t i = 1; i < waitFor.size(); ++i) {
            if (waitFor[i].revents == 0) continue;
            Client &client = clients[i - 1];

            char chunk[4096];
            const ssize_t n = recv(client.fd, chunk, sizeof(chunk), 0);
            if (n <= 0 || client.buffer.size() > 1 << 20) {
                close(client.fd);
                client.fd = -1;
                continue;
            }
            client.buffer.append(chunk, static_cast<size_t>(n));

            size_t newline;
            while ((newline = client.buffer.find('\n')) != std::string::npos) {
                const std::string line = client.buffer.substr(0, newline);
                client.buffer.erase(0, newline + 1);
                if (line.find_first_not_of(" \t\r") == std::string::npos) continue;

                const json request = json::parse(line, nullptr, false);
                json reply = request.is_discarded() ? failure("request is not valid JSON") : handle(request);
                if (!request.is_discarded() && request.is_object() && request.contains("id")) {
                    reply["id"] = request["id"];
                }
                sendLine(client.fd, reply);
            }
        }

        clients.erase(
            std::remove_if(clients.begin(), clients.end(), [](const Client &c) { return c.fd < 0; }),
            clients.end());
    }

    for (const Client &client : clients) close(client.fd);
}

json ControlServer::handle(const json &request) {
    if (!request.is_object() || !request.contains("cmd") || !request["cmd"].is_string()) {
        return failure("request needs a \"cmd\" string");
    }

    const std::string cmd = request["cmd"].get<std::string>();
    if (cmd == "status") return status();
    if (cmd == "get_settings") {
        std::lock_guard<std::mutex> lock(m_settingsMutex);
        return { { "ok", true }, { "settings", m_settings.toJson() } };
    }
    if (cmd == "set") return setSettings(request);
    if (cmd == "load") return load(request);
    if (cmd == "control") return control(request);
    if (cmd == "synth") return synth(request);
    if (cmd == "idle_air") return idleAir(request);
    if (cmd == "generate") return generate();
    if (cmd == "shutdown") {
        if (!m_shutdownHandler) return failure("shutdown isn't supported here");
        m_shutdownHandler();
        return { { "ok", true } };
    }
    if (cmd == "retune") {
        auto lock = m_host->lock();
        if (m_host->bridge() == nullptr) return failure("the BeamNG bridge isn't running");
        m_host->bridge()->retune();
        return { { "ok", true } };
    }

    return failure("unknown command: " + cmd);
}

json ControlServer::status() {
    json reply = { { "ok", true } };
    {
        auto lock = m_host->lock();
        Engine *engine = m_host->engine();
        Simulator *simulator = m_host->simulator();

        if (engine != nullptr && simulator != nullptr) {
            reply["engine"] = {
                { "name", engine->getName() },
                { "script", m_host->loadedScript() },
                { "rpm", engine->getRpm() },
                { "redline_rpm", units::toRpm(engine->getRedline()) },
                { "throttle", engine->getSpeedControl() },
                { "ignition", engine->getIgnitionModule()->m_enabled },
                { "starter", simulator->m_starterMotor.m_enabled },
                { "simulation_frequency", simulator->getSimulationFrequency() },
            };
            if (engine->getIntakeCount() > 0) {
                const Intake *intake = engine->getIntake(0);
                reply["engine"]["idle_air"] = (1 - intake->getIdleThrottlePlatePosition()) * 100;
                reply["engine"]["idle_air_default"] = (1 - intake->getDefaultIdleThrottlePlatePosition()) * 100;
            }
            // Manifold pressure above ambient (negative: vacuum), and any turbos' speeds
            reply["engine"]["manifold_kpa"] = (engine->getManifoldPressure() - units::pressure(1.0, units::atm)) / 1000.0;
            json shafts = json::array();
            for (int i = 0; i < engine->getIntakeCount(); ++i) {
                if (const ForcedInduction *fi = engine->getIntake(i)->getForcedInduction()) {
                    for (int j = 0; j < fi->stageCount(); ++j) shafts.push_back(units::toRpm(fi->state(j).speed) / 1000.0);
                }
            }
            if (!shafts.empty()) reply["engine"]["compressor_krpm"] = shafts;
            reply["engine"]["has_jake_brake"] = engine->getJakeBrakeFlow() > 0;
            reply["engine"]["jake_brake_active"] = engine->isJakeBrakeActive();
        }
        else {
            reply["engine"] = nullptr;
        }

        if (simulator != nullptr) {
            const Synthesizer::AudioParameters audio = simulator->synthesizer().getAudioParameters();
            reply["synth"] = {
                { "volume", audio.volume },
                { "convolution", audio.convolution },
                { "high_freq_gain", audio.dF_F_mix },
                { "low_freq_noise", audio.airNoise },
                { "high_freq_noise", audio.inputSampleNoise },
            };
        }

        BeamBridge *bridge = m_host->bridge();
        reply["bridge"] = {
            { "listening", bridge != nullptr },
            { "linked", bridge != nullptr && bridge->active() },
            { "tuning", bridge != nullptr && bridge->tuning() },
        };
    }

    // What BeamNG says about the player's car (doesn't need the host lock)
    reply["beamng_vehicle"] = nullptr;
    if (BridgeTransport *transport = m_host->bridgeTransport()) {
        json vehicle;
        double age = 0;
        if (transport->vehicleInfo(&vehicle, &age) && age < 10) {
            vehicle["age"] = age;
            reply["beamng_vehicle"] = std::move(vehicle);
        }
    }

    reply["last_build"] = { { "id", m_buildId }, { "ok", m_buildOk }, { "message", m_buildMessage } };

    reply["performance"] = {
        { "steps_per_second", m_host->stepsPerSecond() },
        { "load", m_host->load() },
    };

    std::lock_guard<std::mutex> lock(m_settingsMutex);
    reply["settings"] = m_settings.toJson();
    return reply;
}

json ControlServer::setSettings(const json &request) {
    if (!request.contains("settings")) return failure("set needs \"settings\"");
    if (request["settings"].is_object() && request["settings"].contains("script")) {
        return failure("use the load command to change the engine");
    }

    HostSettings updated;
    {
        std::lock_guard<std::mutex> lock(m_settingsMutex);
        updated = m_settings;
    }

    std::string error;
    if (!updated.merge(request["settings"], &error)) return failure(error);

    applySettings(m_host, updated);
    {
        std::lock_guard<std::mutex> lock(m_settingsMutex);
        m_settings = updated;
    }
    saveSettings();

    std::lock_guard<std::mutex> lock(m_settingsMutex);
    return { { "ok", true }, { "settings", m_settings.toJson() } };
}

json ControlServer::load(const json &request) {
    std::string script;
    if (request.contains("script")) {
        if (!request["script"].is_string()) return failure("script must be a string");
        script = request["script"].get<std::string>();
    }
    if (!script.empty() && !fs::path(script).is_absolute()) {
        return failure("script must be an absolute path");
    }
    if (!script.empty() && !fs::exists(script)) {
        return failure("no such file: " + script);
    }

    return loadAndRemember(script);
}

json ControlServer::loadAndRemember(const std::string &script) {
    // The owner thread swaps engines (the GUI rebuilds its view of the engine at the same time)
    auto promise = std::make_shared<std::promise<EngineHost::LoadResult>>();
    std::future<EngineHost::LoadResult> future = promise->get_future();
    m_host->post([this, promise, script] { promise->set_value(m_host->loadScript(script)); });

    if (future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
        return failure("timed out waiting for the engine to load");
    }

    const EngineHost::LoadResult result = future.get();
    const bool loaded = result.replaced && !result.usedFallback;
    if (loaded) {
        {
            std::lock_guard<std::mutex> lock(m_settingsMutex);
            m_settings.script = script;
        }
        saveSettings();
    }

    json reply = {
        { "ok", loaded },
        { "message", result.message },
    };
    if (!loaded) reply["error"] = result.message;
    if (!result.errors.empty()) reply["errors"] = result.errors;
    return reply;
}

json ControlServer::control(const json &request) {
    auto lock = m_host->lock();
    Engine *engine = m_host->engine();
    Simulator *simulator = m_host->simulator();
    if (engine == nullptr || simulator == nullptr) return failure("no engine loaded");
    if (m_host->bridgeActive()) return failure("BeamNG is driving the engine");

    // Validate everything before changing anything
    for (const char *key : { "ignition", "starter" }) {
        if (request.contains(key) && !request[key].is_boolean()) return failure(std::string(key) + " must be true or false");
    }
    if (request.contains("throttle")) {
        if (!request["throttle"].is_number()) return failure("throttle must be a number");
        const double throttle = request["throttle"].get<double>();
        if (throttle < 0 || throttle > 1) return failure("throttle must be between 0 and 1");
    }

    if (request.contains("ignition")) engine->getIgnitionModule()->m_enabled = request["ignition"].get<bool>();
    if (request.contains("starter")) simulator->m_starterMotor.m_enabled = request["starter"].get<bool>();
    if (request.contains("throttle")) engine->setSpeedControl(request["throttle"].get<double>());

    return { { "ok", true } };
}

// Idle air, percent past the closed throttle (the idle screw), on all intakes
json ControlServer::idleAir(const json &request) {
    const bool reset = request.value("reset", false);
    double percent = 0;
    if (!reset) {
        if (!request.contains("percent") || !request["percent"].is_number()) return failure("idle_air needs \"percent\" or \"reset\"");
        percent = request["percent"].get<double>();
        if (percent < 0 || percent > 50) return failure("idle air must be between 0 and 50 percent");
    }

    auto lock = m_host->lock();
    Engine *engine = m_host->engine();
    if (engine == nullptr) return failure("no engine loaded");
    for (int i = 0; i < engine->getIntakeCount(); ++i) {
        Intake *intake = engine->getIntake(i);
        intake->setIdleThrottlePlatePosition(reset ? intake->getDefaultIdleThrottlePlatePosition() : 1 - percent / 100);
    }
    return { { "ok", true } };
}

json ControlServer::synth(const json &request) {
    // The keys the engine-sim window's Z/X/C/V/B + scroll controls change, all 0 to 1
    static const struct { const char *key; float Synthesizer::AudioParameters::*field; } fields[] = {
        { "volume", &Synthesizer::AudioParameters::volume },
        { "convolution", &Synthesizer::AudioParameters::convolution },
        { "high_freq_gain", &Synthesizer::AudioParameters::dF_F_mix },
        { "low_freq_noise", &Synthesizer::AudioParameters::airNoise },
        { "high_freq_noise", &Synthesizer::AudioParameters::inputSampleNoise },
    };

    for (const auto &f : fields) {
        if (!request.contains(f.key)) continue;
        if (!request[f.key].is_number()) return failure(std::string(f.key) + " must be a number");
        const double value = request[f.key].get<double>();
        if (value < 0 || value > 1) return failure(std::string(f.key) + " must be between 0 and 1");
    }

    auto lock = m_host->lock();
    Engine *engine = m_host->engine();
    Simulator *simulator = m_host->simulator();
    if (engine == nullptr || simulator == nullptr) return failure("no engine loaded");

    Synthesizer::AudioParameters audio = simulator->synthesizer().getAudioParameters();
    if (request.contains("reset") && request["reset"].is_boolean() && request["reset"].get<bool>()) {
        // What loading the engine sets up
        const Synthesizer::AudioParameters defaults;
        audio.volume = defaults.volume;
        audio.convolution = defaults.convolution;
        audio.inputSampleNoise = static_cast<float>(engine->getInitialJitter());
        audio.airNoise = static_cast<float>(engine->getInitialNoise());
        audio.dF_F_mix = static_cast<float>(engine->getInitialHighFrequencyGain());
        audio.convolution = static_cast<float>(engine->getInitialConvolution());
    }
    for (const auto &f : fields) {
        if (request.contains(f.key)) audio.*(f.field) = static_cast<float>(request[f.key].get<double>());
    }
    simulator->synthesizer().setAudioParameters(audio);

    return { { "ok", true } };
}

json ControlServer::generate() {
    json reply = buildEngine();
    m_buildId += 1;
    m_buildOk = reply.value("ok", false);
    m_buildMessage = reply.value(m_buildOk ? "message" : "error", std::string());
    return reply;
}

json ControlServer::buildEngine() {
    BridgeTransport *transport = m_host->bridgeTransport();
    if (transport == nullptr) return failure("the BeamNG bridge isn't running");

    json vehicle;
    double age = 0;
    if (!transport->vehicleInfo(&vehicle, &age)) return failure("BeamNG hasn't described a car yet; get in one first");

    EngineSpec spec;
    std::string error;
    if (!EngineSpec::fromVehicleInfo(vehicle, &spec, &error)) return failure(error);

    const std::string path = saveGeneratedEngine(m_host->paths().userData, vehicle, generateEngineScript(spec), &error);
    if (path.empty()) return failure(error);

    json reply = loadAndRemember(path);
    if (reply.value("ok", false)) {
        char summary[160];
        std::snprintf(summary, sizeof(summary), "Built %s: %.1f mm bore, %.1f mm stroke, %.0f rpm limit",
            spec.name.c_str(), spec.bore * 1000, spec.stroke * 1000, spec.revLimitRpm);
        std::string message = summary;
        for (const std::string &note : spec.notes) message += "\n- " + note;
        reply["message"] = message;
        reply["script"] = path;
    }
    return reply;
}

// With auto_engine on, a new car (or a new engine in it) gets a matching engine built
void ControlServer::maybeAutoGenerate() {
    {
        std::lock_guard<std::mutex> lock(m_settingsMutex);
        if (!m_settings.bridge.autoEngine) {
            m_lastAutoCar.clear(); // Turning it on builds for the current car straight away
            return;
        }
    }

    BridgeTransport *transport = m_host->bridgeTransport();
    json vehicle;
    double age = 0;
    if (transport == nullptr || !transport->vehicleInfo(&vehicle, &age) || age > 10) return;

    const json car = vehicle.value("vehicle", json::object());
    const json engine = vehicle.value("engine", json());
    const std::string key = car.value("directory", std::string()) + "|" + car.value("configuration", std::string())
        + "|" + (engine.is_object() ? engine.value("part", std::string()) : std::string("none"));
    if (key == m_lastAutoCar) return;
    m_lastAutoCar = key;

    if (!engine.is_object()) return; // Electric: nothing to build
    const json result = generate();
    std::fprintf(stderr, "engine-sim: automatic engine: %s\n",
        result.value("message", result.value("error", std::string("?"))).c_str());
}

void ControlServer::saveSettings() {
    HostSettings copy;
    {
        std::lock_guard<std::mutex> lock(m_settingsMutex);
        copy = m_settings;
    }

    std::string error;
    if (!copy.save(m_settingsFile, &error)) {
        std::cerr << "engine-sim: " << error << std::endl;
    }
}
