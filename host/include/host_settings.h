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
    // Lowers the synthesizer's level so its peaks don't clip. Upstream (0 dB) clips 1.5-3.5% of
    // samples; 3 dB nearly stops it and 6 dB does.
    double headroomDb = 6;
    // How long the leveler holds a loud peak, seconds. Upstream's 0.023 s turns the quiet between
    // firing pulses up (idle as loud as full throttle); seconds leave a quiet idle quiet for a long
    // time after revving. The leveler is frozen while cranking or stopped, which is what stops the
    // hissy "record scratch" there.
    double levelerRelease = 0.15;
    // Turbo and supercharger sounds (whine, hiss, blow-off, flutter) relative to the engine
    double turboVolume = 1.0;
    // A diesel's compression-release brake, when it has one (the cab switch). Off by default:
    // it braps on every lift, free revs included
    bool jakeBrake = false;
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
