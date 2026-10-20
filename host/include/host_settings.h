#ifndef ATG_ENGINE_SIM_HOST_HOST_SETTINGS_H
#define ATG_ENGINE_SIM_HOST_HOST_SETTINGS_H

#include "beam_bridge.h"

#include <nlohmann/json.hpp>

#include <string>

class EngineHost;

// Settings that survive restarts, stored as JSON in ~/.config/engine-sim/settings.json
struct HostSettings {
    std::string script;                 // Engine script to load; empty for the default main.mr
    double simulationFrequency = 0;     // Overrides the engine's own rate when non-zero (Hz)
    BeamBridge::Settings bridge;

    nlohmann::json toJson() const;

    // Applies the fields present in `json` on top of these settings. Unknown keys are ignored;
    // wrong types or out-of-range values fail with `error` set and nothing changed.
    bool merge(const nlohmann::json &json, std::string *error);

    // Missing file: defaults. Unreadable or invalid: defaults, with `error` set.
    static HostSettings load(const std::string &path, std::string *error);
    bool save(const std::string &path, std::string *error) const;
};

// Pushes the live parts (bridge, simulation rate) into a running host. Takes the host lock.
void applySettings(EngineHost *host, const HostSettings &settings);

#endif /* ATG_ENGINE_SIM_HOST_HOST_SETTINGS_H */
