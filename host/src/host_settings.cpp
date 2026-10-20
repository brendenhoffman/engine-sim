#include "host_settings.h"

#include "engine_host.h"

#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;
using nlohmann::json;

namespace {
    // Reads an optional field, checking its type and range
    template <typename T>
    bool readField(const json &object, const char *key, T *out, std::string *error, double min = -1e300, double max = 1e300) {
        const auto it = object.find(key);
        if (it == object.end()) return true;

        if constexpr (std::is_same_v<T, bool>) {
            if (!it->is_boolean()) { *error = std::string(key) + " must be true or false"; return false; }
            *out = it->get<bool>();
        }
        else if constexpr (std::is_same_v<T, std::string>) {
            if (!it->is_string()) { *error = std::string(key) + " must be a string"; return false; }
            *out = it->get<std::string>();
        }
        else {
            if (!it->is_number()) { *error = std::string(key) + " must be a number"; return false; }
            const double value = it->get<double>();
            if (value < min || value > max) {
                *error = std::string(key) + " must be between " + std::to_string(min) + " and " + std::to_string(max);
                return false;
            }
            *out = static_cast<T>(value);
        }

        return true;
    }
}

json HostSettings::toJson() const {
    return {
        { "script", script },
        { "simulation_frequency", simulationFrequency },
        { "audio", { { "headroom_db", headroomDb } } },
        { "bridge", {
            { "connect", bridge.connect },
            { "drive_car", bridge.driveCar },
            { "volume", bridge.volume },
            { "beam_idle_rpm", bridge.beamIdleRpm },
            { "tune_beam_engine", bridge.tuneBeamEngine },
            { "neutral_eq", bridge.neutralEq },
            { "intake_sound", bridge.intakeSound },
            { "exhaust_sound", bridge.exhaustSound },
            { "game_low_cut", bridge.keepGameLowCut },
        } },
    };
}

bool HostSettings::merge(const json &object, std::string *error) {
    if (!object.is_object()) {
        *error = "settings must be an object";
        return false;
    }

    HostSettings updated = *this;
    if (!readField(object, "script", &updated.script, error)) return false;
    if (!readField(object, "simulation_frequency", &updated.simulationFrequency, error, 0, 400000)) return false;

    const auto audioIt = object.find("audio");
    if (audioIt != object.end()) {
        if (!audioIt->is_object()) {
            *error = "audio must be an object";
            return false;
        }
        if (!readField(*audioIt, "headroom_db", &updated.headroomDb, error, 0, 24)) return false;
    }

    const auto bridgeIt = object.find("bridge");
    if (bridgeIt != object.end()) {
        if (!bridgeIt->is_object()) {
            *error = "bridge must be an object";
            return false;
        }
        if (!readField(*bridgeIt, "connect", &updated.bridge.connect, error)) return false;
        if (!readField(*bridgeIt, "drive_car", &updated.bridge.driveCar, error)) return false;
        if (!readField(*bridgeIt, "volume", &updated.bridge.volume, error, 0, 4)) return false;
        if (!readField(*bridgeIt, "beam_idle_rpm", &updated.bridge.beamIdleRpm, error, 0, 20000)) return false;
        if (!readField(*bridgeIt, "tune_beam_engine", &updated.bridge.tuneBeamEngine, error)) return false;
        if (!readField(*bridgeIt, "neutral_eq", &updated.bridge.neutralEq, error)) return false;
        if (!readField(*bridgeIt, "intake_sound", &updated.bridge.intakeSound, error)) return false;
        if (!readField(*bridgeIt, "exhaust_sound", &updated.bridge.exhaustSound, error)) return false;
        if (!readField(*bridgeIt, "game_low_cut", &updated.bridge.keepGameLowCut, error)) return false;
    }

    if (updated.simulationFrequency != 0 && updated.simulationFrequency < 400) {
        *error = "simulation_frequency must be 0 (engine default) or at least 400";
        return false;
    }

    *this = updated;
    return true;
}

HostSettings HostSettings::load(const std::string &path, std::string *error) {
    HostSettings settings;
    if (!fs::exists(path)) return settings;

    std::ifstream file(path);
    const json parsed = json::parse(file, nullptr, false);
    if (parsed.is_discarded()) {
        *error = path + " is not valid JSON; using defaults";
        return settings;
    }

    std::string mergeError;
    if (!settings.merge(parsed, &mergeError)) {
        *error = path + ": " + mergeError + "; using defaults";
        return HostSettings();
    }

    return settings;
}

bool HostSettings::save(const std::string &path, std::string *error) const {
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);

    // Write then rename, so a crash can't leave a half-written file
    const std::string temporary = path + ".tmp";
    {
        std::ofstream file(temporary);
        file << toJson().dump(2) << '\n';
        if (!file) {
            *error = "could not write " + temporary;
            return false;
        }
    }

    fs::rename(temporary, path, ec);
    if (ec) {
        *error = "could not replace " + path + ": " + ec.message();
        return false;
    }

    return true;
}

void applySettings(EngineHost *host, const HostSettings &settings) {
    auto lock = host->lock();

    if (host->bridge() != nullptr) {
        host->bridge()->settings() = settings.bridge;
    }

    host->setSimulationFrequencyOverride(settings.simulationFrequency);
    host->setOutputHeadroom(settings.headroomDb);
}
