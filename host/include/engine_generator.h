#ifndef ATG_ENGINE_SIM_HOST_ENGINE_GENERATOR_H
#define ATG_ENGINE_SIM_HOST_ENGINE_GENERATOR_H

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

// Everything needed to write an engine-sim engine, worked out from what BeamNG knows about a car's
// engine plus estimates for the rest. Lengths in metres, angles in degrees.
struct EngineSpec {
    enum class Layout { Inline, V, Flat };

    std::string name;              // Shown in engine-sim
    std::string sourceName;        // BeamNG's name for the engine
    Layout layout = Layout::Inline;
    int cylinders = 4;
    double bankAngle = 0;          // V angle (Flat engines are 180)
    bool flatPlaneCrank = false;   // V8 only

    double displacementLitres = 2.0;
    double bore = 0.086;
    double stroke = 0.086;
    double compressionRatio = 10.5;

    double idleRpm = 800;
    double redlineRpm = 6500;
    double revLimitRpm = 6800;

    bool diesel = false;
    bool forcedInduction = false;  // engine-sim can't model it; noted so the user knows

    // Per cylinder, in bank order: which bank, and the crank angle it fires at (0-720)
    struct Cylinder {
        int bank;
        int indexInBank;
        double firingAngle;
    };
    std::vector<Cylinder> firing;

    // How each value was arrived at, for the user ("displacement 6.9 L from the name")
    std::vector<std::string> notes;

    int bankCount() const { return layout == Layout::Inline ? 1 : 2; }
    double bankAngleOf(int bank) const;
    std::string layoutName() const;  // "V8", "I4", "flat-6"

    // From the "engine" part of a BeamNG vehicle description ("BESV"). False (with a reason)
    // when there's nothing to build from, e.g. an electric car.
    static bool fromVehicleInfo(const nlohmann::json &vehicle, EngineSpec *spec, std::string *error);

    // Fills bore/stroke and the firing order from the other fields
    void finish();
};

// A complete engine-sim script (with main()) for the spec
std::string generateEngineScript(const EngineSpec &spec);

// Writes the script to <userData>/generated/<car>_<engine part>.mr and returns the path,
// or an empty string (with `error` set) if it couldn't be written
std::string saveGeneratedEngine(const std::string &userData, const nlohmann::json &vehicle,
    const std::string &script, std::string *error);

#endif /* ATG_ENGINE_SIM_HOST_ENGINE_GENERATOR_H */
