#include "engine_generator.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <cstdio>
#include <regex>
#include <sstream>

using nlohmann::json;

// Most values are scaled from the GM LS script that ships with engine-sim (data/assets/engines/
// atg-video-2/07_gm_ls.mr), a well-tuned, known-good engine:
namespace reference {
    constexpr double Bore = 0.096;            // 3.78 in
    constexpr double Stroke = 0.092;          // 3.622 in
    constexpr double Displacement = 5.33;     // L
    constexpr double CylinderVolume = Displacement / 8;
    constexpr double RevLimit = 6800;
    constexpr double PI = 3.14159265358979323846;

    // Flow bench, CFM at 28 inH2O, every 50 thou of lift (2-valve head)
    constexpr double IntakeFlow[] = { 0, 1, 103, 156, 214, 249, 268, 280, 280, 281 };
    constexpr double ExhaustFlow[] = { 0, 1, 72, 113, 160, 196, 222, 235, 245, 246 };
}

namespace {
    double clampValue(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }

    std::string lower(std::string text) {
        std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
        return text;
    }

    template <typename T>
    T read(const json &object, const char *key, T fallback) {
        if (!object.is_object()) return fallback;
        const auto it = object.find(key);
        if (it == object.end()) return fallback;
        if constexpr (std::is_same_v<T, std::string>) return it->is_string() ? it->template get<std::string>() : fallback;
        else return it->is_number() ? it->template get<T>() : fallback;
    }

    std::string fmt(const char *format, double value) {
        char buffer[64];
        std::snprintf(buffer, sizeof(buffer), format, value);
        return buffer;
    }

    std::string mm(double metres) { return fmt("%.2f", metres * 1000) + " * units.mm"; }
    std::string deg(double degrees) { return fmt("%.2f", degrees) + " * units.deg"; }
    std::string rpm(double value) { return fmt("%.0f", value) + " * units.rpm"; }

    std::string escape(const std::string &text) {
        std::string out;
        for (char c : text) {
            if (c == '"' || c == '\\') out += '\\';
            if (c != '\n') out += c;
        }
        return out;
    }

    // Conventional firing orders, by cylinder number. `bankOf` maps a cylinder number to
    // (bank, index in bank) for that numbering convention.
    struct FiringOrder {
        std::vector<int> order;
        bool oddEvenBanks; // V6/V8 (GM) and flat-4 number alternately; others number bank A first
    };

    FiringOrder conventionalOrder(EngineSpec::Layout layout, int cylinders, bool flatPlane) {
        using L = EngineSpec::Layout;
        if (layout == L::Inline) {
            switch (cylinders) {
                case 3: return { { 1, 2, 3 }, false };
                case 4: return { { 1, 3, 4, 2 }, false };
                case 5: return { { 1, 2, 4, 5, 3 }, false };
                case 6: return { { 1, 5, 3, 6, 2, 4 }, false };
                case 8: return { { 1, 6, 2, 5, 8, 3, 7, 4 }, false };
                default: break;
            }
        }
        else if (layout == L::V) {
            switch (cylinders) {
                case 6: return { { 1, 2, 3, 4, 5, 6 }, true };
                case 8:
                    return flatPlane
                        ? FiringOrder{ { 1, 5, 3, 7, 4, 8, 2, 6 }, false }
                        : FiringOrder{ { 1, 8, 4, 3, 6, 5, 7, 2 }, true };
                case 10: return { { 1, 6, 5, 10, 2, 7, 3, 8, 4, 9 }, false };
                case 12: return { { 1, 7, 5, 11, 3, 9, 6, 12, 2, 8, 4, 10 }, false };
                default: break;
            }
        }
        else if (layout == L::Flat) {
            switch (cylinders) {
                case 4: return { { 1, 3, 2, 4 }, true };
                case 6: return { { 1, 6, 2, 4, 3, 5 }, false };
                default: break;
            }
        }

        // Anything else: even firing, alternating banks
        FiringOrder generic{ {}, layout != L::Inline };
        for (int i = 1; i <= cylinders; ++i) generic.order.push_back(i);
        return generic;
    }
}

double EngineSpec::bankAngleOf(int bank) const {
    switch (layout) {
        case Layout::Inline: return 0;
        case Layout::Flat: return bank == 0 ? 90 : -90;
        case Layout::V: return bank == 0 ? -bankAngle / 2 : bankAngle / 2;
    }
    return 0;
}

std::string EngineSpec::layoutName() const {
    switch (layout) {
        case Layout::Inline: return "I" + std::to_string(cylinders);
        case Layout::V: return "V" + std::to_string(cylinders);
        case Layout::Flat: return "flat-" + std::to_string(cylinders);
    }
    return "?";
}

void EngineSpec::finish() {
    if ((layout == Layout::V || layout == Layout::Flat) && cylinders % 2 != 0) {
        notes.push_back("an odd cylinder count can't be split into two banks; built as an inline");
        layout = Layout::Inline;
    }

    // Bore/stroke: high-revving engines are oversquare, diesels and low revvers undersquare
    double boreToStroke = 1.0;
    if (diesel) boreToStroke = 0.85;
    else if (revLimitRpm >= 8000) boreToStroke = 1.25;
    else if (revLimitRpm >= 7000) boreToStroke = 1.1;
    else if (revLimitRpm <= 5000) boreToStroke = 0.92;

    const double cylinderVolume = displacementLitres / 1000.0 / cylinders; // m^3
    bore = std::cbrt(4 * cylinderVolume * boreToStroke / reference::PI);
    stroke = bore / boreToStroke;
    compressionRatio = diesel ? 16.0 : (revLimitRpm >= 7500 ? 11.0 : 10.5);

    // Firing angles: even intervals over the 720 degree cycle in conventional order
    const FiringOrder order = conventionalOrder(layout, cylinders, flatPlaneCrank);
    const int perBank = layout == Layout::Inline ? cylinders : cylinders / 2;
    firing.assign(cylinders, Cylinder{});
    for (int k = 0; k < cylinders; ++k) {
        const int number = order.order[k]; // 1-based
        Cylinder cylinder;
        if (layout == Layout::Inline) {
            cylinder.bank = 0;
            cylinder.indexInBank = number - 1;
        }
        else if (order.oddEvenBanks) {
            cylinder.bank = (number % 2 == 1) ? 0 : 1;
            cylinder.indexInBank = (number - 1) / 2;
        }
        else {
            cylinder.bank = (number <= perBank) ? 0 : 1;
            cylinder.indexInBank = (number - 1) % perBank;
        }
        cylinder.firingAngle = k * 720.0 / cylinders;
        firing[cylinder.bank * perBank + cylinder.indexInBank] = cylinder;
    }
}

bool EngineSpec::fromVehicleInfo(const json &vehicle, EngineSpec *out, std::string *error) {
    const json engine = vehicle.is_object() && vehicle.contains("engine") ? vehicle["engine"] : json();
    if (!engine.is_object()) {
        *error = "this car has no combustion engine to build from";
        return false;
    }

    EngineSpec spec;
    spec.sourceName = read(engine, "name", read(engine, "part", std::string("BeamNG engine")));
    const std::string name = lower(read(engine, "name", std::string()));
    const std::string part = lower(read(engine, "part", std::string()));
    // BeamNG's sound samples encode the engine: "i6dt_12800cc_stock_...", "V10_2_exhaust", "i4p_01600cc_..."
    const std::string samples = lower(read(engine, "exhaust_sample", std::string()) + " " + read(engine, "engine_sample", std::string()));
    const std::string text = name + " " + part;
    std::smatch match;

    // --- Layout and cylinders, from the name or part ("6.9L V8", "pickup_engine_v8_6.9")
    int namedCylinders = 0;
    if (std::regex_search(text, match, std::regex(R"((?:^|[^a-z])v-?(\d{1,2})(?![0-9.]))"))) {
        spec.layout = Layout::V;
        namedCylinders = std::stoi(match[1]);
    }
    else if (std::regex_search(text, match, std::regex(R"((?:^|[^a-z])w-?(\d{1,2})(?![0-9.]))"))) {
        spec.layout = Layout::V;
        namedCylinders = std::stoi(match[1]);
        spec.notes.push_back("W engines are built as a V with the same cylinder count");
    }
    else if (std::regex_search(text, match, std::regex(R"((?:flat|boxer|(?:^|[^a-z])h)[- ]?(\d{1,2})(?![0-9.]))"))) {
        spec.layout = Layout::Flat;
        namedCylinders = std::stoi(match[1]);
    }
    else if (std::regex_search(text, match, std::regex(R"((?:inline|straight|(?:^|[^a-z])[il])[- ]?(\d{1,2})(?![0-9.]))"))) {
        spec.layout = Layout::Inline;
        namedCylinders = std::stoi(match[1]);
    }

    std::string layoutSource = "the name";
    if (namedCylinders == 0 && std::regex_search(samples, match, std::regex(R"((?:^|[^a-z])([ivh])(\d{1,2})(?=[a-z_]))"))) {
        const char kind = match[1].str()[0];
        spec.layout = kind == 'v' ? Layout::V : kind == 'h' ? Layout::Flat : Layout::Inline;
        namedCylinders = std::stoi(match[2]);
        layoutSource = "the sound samples";
    }

    const int soundCylinders = read(engine, "cylinders", 0);
    if (namedCylinders > 0) {
        spec.cylinders = namedCylinders;
        spec.notes.push_back("layout " + std::string(spec.layout == Layout::V ? "V" : spec.layout == Layout::Flat ? "flat" : "inline")
            + std::to_string(namedCylinders) + " from " + layoutSource);
    }
    else if (soundCylinders > 0) {
        spec.cylinders = soundCylinders;
        spec.layout = soundCylinders >= 6 && soundCylinders % 2 == 0 && soundCylinders != 6 ? Layout::V : Layout::Inline;
        spec.notes.push_back(std::to_string(soundCylinders) + " cylinders from the sound setup; layout guessed");
    }
    else {
        spec.cylinders = 4;
        spec.notes.push_back("cylinder count unknown; guessed an inline-4");
    }
    if (name.find("rotary") != std::string::npos || name.find("wankel") != std::string::npos) {
        spec.notes.push_back("rotary engines can't be simulated; built a piston engine instead");
    }
    spec.cylinders = std::clamp(spec.cylinders, 1, 16);

    // --- Fuel and induction
    spec.diesel = read(engine, "fuel", std::string()) == "diesel" || name.find("diesel") != std::string::npos;
    spec.forcedInduction = vehicle.contains("turbo") || vehicle.contains("supercharger")
        || name.find("turbo") != std::string::npos || name.find("supercharg") != std::string::npos;
    if (spec.diesel) spec.notes.push_back("diesel: engine-sim only burns petrol with a spark, so this is an approximation");
    if (spec.forcedInduction) spec.notes.push_back("forced induction can't be simulated; built naturally aspirated");

    // --- Revs
    spec.idleRpm = read(engine, "idle_rpm", 800.0);
    spec.revLimitRpm = read(engine, "rev_limiter_rpm", read(engine, "max_rpm", 6500.0));
    spec.redlineRpm = read(engine, "max_rpm", spec.revLimitRpm);
    if (spec.redlineRpm > spec.revLimitRpm) spec.redlineRpm = spec.revLimitRpm * 0.97;
    spec.revLimitRpm = clampValue(spec.revLimitRpm, 1500, 20000);
    spec.idleRpm = clampValue(spec.idleRpm, 400, spec.revLimitRpm * 0.5);

    // --- Displacement: the name, the part ("_6.9"), or an estimate from peak torque
    double displacement = 0;
    if (std::regex_search(name, match, std::regex(R"((\d+(?:\.\d+)?)\s*l(?![a-z]))"))) {
        displacement = std::stod(match[1]);
        spec.notes.push_back("displacement " + fmt("%.1f", displacement) + " L from the name");
    }
    else if (std::regex_search(part, match, std::regex(R"(_(\d{1,2}\.\d)(?:_|$))"))) {
        displacement = std::stod(match[1]);
        spec.notes.push_back("displacement " + fmt("%.1f", displacement) + " L from the part name");
    }
    else if (std::regex_search(samples, match, std::regex(R"((\d{3,5})cc)"))) {
        displacement = std::stod(match[1]) / 1000.0;
        spec.notes.push_back("displacement " + fmt("%.1f", displacement) + " L from the sound samples");
    }
    else {
        double peak = 0;
        for (const json &row : engine.value("torque", json::array())) {
            if (row.is_array() && row.size() >= 2 && row[1].is_number()) peak = std::max(peak, row[1].get<double>());
        }
        if (peak > 0) {
            displacement = peak / (spec.diesel ? 150.0 : 100.0);
            spec.notes.push_back("displacement " + fmt("%.1f", displacement) + " L estimated from peak torque");
        }
        else {
            displacement = 0.5 * spec.cylinders;
            spec.notes.push_back("displacement unknown; guessed 0.5 L per cylinder");
        }
    }
    spec.displacementLitres = clampValue(displacement, 0.1 * spec.cylinders, 2.5 * spec.cylinders);

    // --- Bank angle and crank
    if (spec.layout == Layout::Flat) spec.bankAngle = 180;
    else if (spec.layout == Layout::V) {
        switch (spec.cylinders) {
            case 6: spec.bankAngle = 60; break;
            case 12: spec.bankAngle = 60; break;
            default: spec.bankAngle = 90; break;
        }
        std::smatch angle;
        if (std::regex_search(name, angle, std::regex(R"((\d{2,3})\s*(?:°|deg))"))) spec.bankAngle = std::stod(angle[1]);
    }
    spec.flatPlaneCrank = spec.layout == Layout::V && spec.cylinders == 8
        && (name.find("flat") != std::string::npos || spec.revLimitRpm >= 8500);

    // "Cherrier Percheron 12.8L I6": the car and what was built (some engines' own part names are
    // just the brand)
    const std::string model = read(vehicle.value("vehicle", json::object()), "model", std::string());
    spec.finish();
    spec.name = (model.empty() ? std::string() : model + " ") + fmt("%.1f", spec.displacementLitres) + "L " + spec.layoutName();

    *out = spec;
    return true;
}

std::string generateEngineScript(const EngineSpec &spec) {
    using namespace reference;
    std::ostringstream s;

    const int banks = spec.bankCount();
    const int perBank = spec.cylinders / banks;

    // Straight sixes split into two exhausts of three (front and rear, like the 2JZ and most
    // modern I6s): each pipe then gets an even pulse every 240 degrees, the smooth I6 sound.
    // One shared pipe with unequal runners sounds rumbly.
    const bool splitInline = spec.layout == EngineSpec::Layout::Inline && spec.cylinders == 6;
    const int exhausts = splitInline ? 2 : banks;
    const double cylinderVolume = spec.displacementLitres / spec.cylinders;

    // Scale factors against the LS
    const double b = spec.bore / Bore;                         // linear (head, valves, cam)
    const double v = spec.displacementLitres / Displacement;   // whole engine
    const double c = cylinderVolume / CylinderVolume;          // per cylinder
    const double r = spec.revLimitRpm / RevLimit;              // airflow demand
    const bool fourValve = spec.revLimitRpm >= 6500 && !spec.diesel;
    const double valveFlow = fourValve ? 1.25 : 1.0;

    const double rodLength = 1.7 * spec.stroke;
    const double compressionHeight = 0.26 * spec.bore;
    const double chamberCc = cylinderVolume * 1000.0 / (spec.compressionRatio - 1.0);
    const double camDuration = clampValue(214 + 30 * (spec.revLimitRpm - 5000) / 3000, 200, 272);
    const double camLift = 551 * b * (fourValve ? 0.72 : 1.0); // thou
    const double lobeCenter = spec.revLimitRpm >= 7500 ? 110 : 116;
    const double runnerLength = clampValue(12.0 / r, 5, 16);   // inch

    // Sound settings follow the shipped engines that sound best for each kind of engine (the LS
    // the rest is scaled from is unusually hissy): little high-frequency jitter, a mild exhaust
    // response, and a higher simulation rate for engines with fewer cylinders
    // V8s: the LS's own 0.6 gives modern V8s their bite; old low-revving ones turn to static with it
    double jitter = spec.diesel ? 0.4 : (spec.revLimitRpm >= 8000 ? 0.12 : 0.25);
    if (!spec.diesel && spec.layout == EngineSpec::Layout::V && spec.cylinders == 8) {
        jitter = spec.revLimitRpm >= 8000 ? 0.3 : (spec.revLimitRpm >= 5500 ? 0.6 : 0.35);
    }
    std::string impulse = "mild_exhaust_0_reverb";
    if (spec.cylinders <= 5 || spec.layout == EngineSpec::Layout::Flat) impulse = "mild_exhaust_0";
    if (spec.cylinders >= 12 || (spec.revLimitRpm >= 8500 && spec.cylinders >= 8)) impulse = "minimal_muffling_01";
    int simulationFrequency = 10000;
    if (spec.cylinders <= 3) simulationFrequency = 20000;
    else if (spec.cylinders <= 5) simulationFrequency = 16000;
    else if (spec.cylinders <= 6) simulationFrequency = 12000;
    else if (spec.cylinders > 8) simulationFrequency = 8000;
    const double primaryLength = clampValue(29.0 / r, 12, 36); // inch

    s << "// Generated by engine-sim from BeamNG's \"" << escape(spec.sourceName) << "\".\n"
      << "// Estimates: ";
    for (size_t i = 0; i < spec.notes.size(); ++i) s << (i ? "; " : "") << escape(spec.notes[i]);
    s << ".\n// " << spec.layoutName() << ", " << fmt("%.2f", spec.displacementLitres) << " L, bore "
      << fmt("%.1f", spec.bore * 1000) << " mm, stroke " << fmt("%.1f", spec.stroke * 1000) << " mm.\n"
      << "// Copy this file somewhere else before editing: it's rewritten when regenerated.\n\n"
      << "import \"engine_sim.mr\"\n"
      << "import \"themes/default.mr\"\n\n"
      << "units units()\nconstants constants()\nimpulse_response_library ir_lib()\n\n";

    // --- Ignition wires
    s << "private node generated_wires {\n";
    for (int i = 1; i <= spec.cylinders; ++i) s << "    output wire" << i << ": ignition_wire();\n";
    s << "}\n\n";

    // --- Head (flow scaled from the LS by valve area)
    s << "private node generated_head {\n"
      << "    input intake_camshaft;\n"
      << "    input exhaust_camshaft;\n"
      << "    input flip_display: false;\n"
      << "    alias output __out: head;\n\n"
      << "    function intake_flow(" << fmt("%.1f", 50 * b) << " * units.thou)\n    intake_flow\n";
    for (int i = 0; i < 10; ++i) {
        s << "        .add_flow_sample(" << fmt("%.1f", i * 50 * b) << ", " << fmt("%.1f", IntakeFlow[i] * b * b * valveFlow) << ")\n";
    }
    s << "\n    function exhaust_flow(" << fmt("%.1f", 50 * b) << " * units.thou)\n    exhaust_flow\n";
    for (int i = 0; i < 10; ++i) {
        s << "        .add_flow_sample(" << fmt("%.1f", i * 50 * b) << ", " << fmt("%.1f", ExhaustFlow[i] * b * b * valveFlow) << ")\n";
    }
    s << "\n    generic_cylinder_head head(\n"
      << "        chamber_volume: " << fmt("%.1f", chamberCc) << " * units.cc,\n"
      << "        intake_runner_volume: " << fmt("%.1f", 149.6 * c) << " * units.cc,\n"
      << "        intake_runner_cross_section_area: " << fmt("%.2f", 2.2 * b) << " * units.inch * " << fmt("%.2f", 2.2 * b) << " * units.inch,\n"
      << "        exhaust_runner_volume: " << fmt("%.1f", 50.0 * c) << " * units.cc,\n"
      << "        exhaust_runner_cross_section_area: " << fmt("%.2f", 1.75 * b) << " * units.inch * " << fmt("%.2f", 1.75 * b) << " * units.inch,\n"
      << "        intake_port_flow: intake_flow,\n"
      << "        exhaust_port_flow: exhaust_flow,\n"
      << "        valvetrain: standard_valvetrain(\n"
      << "            intake_camshaft: intake_camshaft,\n"
      << "            exhaust_camshaft: exhaust_camshaft\n"
      << "        ),\n"
      << "        flip_display: flip_display\n"
      << "    )\n"
      << "}\n\n";

    s << "private node generated_flame_speed {\n"
      << "    alias output __out:\n"
      << "        function(5.0)\n"
      << "            .add_sample(0.0, 3.0)\n"
      << "            .add_sample(5.0, 1.5 * 5.0)\n"
      << "            .add_sample(10.0, 1.75 * 10.0)\n";
    for (int i = 15; i <= 45; i += 5) s << "            .add_sample(" << i << ".0, 2.0 * " << i << ".0)\n";
    s << "            ;\n}\n\n";

    // --- The engine
    s << "public node generated_engine {\n"
      << "    alias output __out: engine;\n\n"
      << "    engine engine(\n"
      << "        name: \"" << escape(spec.name) << "\",\n"
      << "        starter_torque: " << fmt("%.0f", 200 * std::max(v, 0.3)) << " * units.lb_ft,\n"
      << "        starter_speed: 200 * units.rpm,\n"
      << "        redline: " << rpm(spec.redlineRpm) << ",\n"
      << "        throttle_gamma: 2.0,\n"
      << "        fuel: fuel(\n"
      << "            max_burning_efficiency: 1.0,\n"
      << "            turbulence_to_flame_speed_ratio: generated_flame_speed()\n"
      << "        ),\n"
      << "        hf_gain: 0.01,\n"
      << "        noise: 1.0,\n"
      << "        jitter: " << fmt("%.2f", jitter) << ",\n"
      << "        simulation_frequency: " << simulationFrequency << "\n"
      << "    )\n\n"
      << "    generated_wires wires()\n\n"
      << "    label stroke(" << mm(spec.stroke) << ")\n"
      << "    label bore(" << mm(spec.bore) << ")\n"
      << "    label rod_length(" << mm(rodLength) << ")\n"
      << "    label rod_mass(" << fmt("%.1f", 50 * std::pow(b, 3)) << " * units.g)\n"
      << "    label compression_height(" << mm(compressionHeight) << ")\n"
      << "    label crank_mass(" << fmt("%.1f", 60 * v) << " * units.lb)\n"
      << "    label flywheel_mass(" << fmt("%.1f", 30 * std::sqrt(v)) << " * units.lb)\n"
      << "    label flywheel_radius(8 * units.inch)\n\n"
      << "    label crank_moment(1.5 * disk_moment_of_inertia(mass: crank_mass, radius: stroke))\n"
      << "    label flywheel_moment(disk_moment_of_inertia(mass: flywheel_mass, radius: flywheel_radius))\n"
      << "    label other_moment(disk_moment_of_inertia(mass: 1 * units.kg, radius: 1.0 * units.cm))\n\n"
      << "    crankshaft c0(\n"
      << "        throw: stroke / 2,\n"
      << "        flywheel_mass: flywheel_mass,\n"
      << "        mass: crank_mass,\n"
      << "        friction_torque: " << fmt("%.1f", 20 * v) << " * units.lb_ft,\n"
      << "        moment_of_inertia: crank_moment + flywheel_moment + other_moment,\n"
      << "        position_x: 0.0,\n"
      << "        position_y: 0.0,\n"
      << "        tdc: 90 * units.deg\n"
      << "    )\n\n";

    // A journal per cylinder, placed so the cylinder reaches TDC at its firing angle:
    // with tdc = 90 deg, a cylinder on a bank at angle B fires at crank angle A - B for a pin at A
    s << "    // Firing order and pins: each cylinder gets its own journal at (firing angle + bank angle)\n";
    for (int i = 0; i < spec.cylinders; ++i) {
        const EngineSpec::Cylinder &cyl = spec.firing[i];
        const double pin = std::fmod(cyl.firingAngle + spec.bankAngleOf(cyl.bank) + 720.0, 360.0);
        s << "    rod_journal rj" << i << "(angle: " << deg(pin) << ") // bank " << cyl.bank
          << " #" << (cyl.indexInBank + 1) << ", fires at " << fmt("%.0f", cyl.firingAngle) << "\n";
    }
    s << "    c0\n";
    for (int i = 0; i < spec.cylinders; ++i) s << "        .add_rod_journal(rj" << i << ")\n";
    s << "\n";

    s << "    piston_parameters piston_params(\n"
      << "        mass: " << fmt("%.1f", 100 * std::pow(b, 3)) << " * units.g,\n"
      << "        compression_height: compression_height,\n"
      << "        wrist_pin_position: 0.0,\n"
      << "        displacement: 0.0\n"
      << "    )\n\n"
      << "    connecting_rod_parameters cr_params(\n"
      << "        mass: rod_mass,\n"
      << "        moment_of_inertia: rod_moment_of_inertia(mass: rod_mass, length: rod_length),\n"
      << "        center_of_mass: 0.0,\n"
      << "        length: rod_length\n"
      << "    )\n\n"
      << "    intake intake(\n"
      << "        plenum_volume: " << fmt("%.3f", 1.325 * v) << " * units.L,\n"
      << "        plenum_cross_section_area: " << fmt("%.1f", 20.0 * std::max(v, 0.25)) << " * units.cm2,\n"
      << "        intake_flow_rate: k_carb(" << fmt("%.1f", 700 * v * r) << "),\n"
      << "        runner_flow_rate: k_carb(" << fmt("%.1f", 100 * c * r) << "),\n"
      << "        runner_length: " << fmt("%.1f", runnerLength) << " * units.inch,\n"
      << "        idle_flow_rate: k_carb(0.0),\n"
      << "        idle_throttle_plate_position: 0.996,\n"
      << "        velocity_decay: 0.5\n"
      << "    )\n\n"
      << "    exhaust_system_parameters es_params(\n"
      << "        outlet_flow_rate: k_carb(" << fmt("%.1f", 1000 * v * r * 2.0 / exhausts) << "),\n"
      << "        primary_tube_length: " << fmt("%.1f", primaryLength) << " * units.inch,\n"
      << "        primary_flow_rate: k_carb(" << fmt("%.1f", 500 * c * r) << "),\n"
      << "        velocity_decay: 1.0\n"
      << "    )\n\n";

    for (int bank = 0; bank < exhausts; ++bank) {
        const int length = exhausts == 1 ? 120 : (splitInline ? 100 : (bank == 0 ? 100 : 140));
        s << "    exhaust_system exhaust" << bank << "(\n"
          << "        es_params,\n"
          << "        audio_volume: " << (splitInline ? "2.5" : "4.0") << ",\n"
          << "        length: " << length << " * units.inch,\n"
          << "        impulse_response: ir_lib." << impulse << "\n"
          << "    )\n";
    }
    s << "\n    cylinder_bank_parameters bank_params(\n"
      << "        bore: bore,\n"
      << "        deck_height: stroke / 2 + rod_length + compression_height\n"
      << "    )\n\n"
      << "    label spacing(" << mm(spec.bore * 1.15) << ")\n";

    for (int bank = 0; bank < banks; ++bank) {
        s << "    cylinder_bank b" << bank << "(bank_params, angle: " << deg(spec.bankAngleOf(bank)) << ")\n";
    }
    s << "\n";

    // Cams: one intake and one exhaust per bank, a lobe per cylinder at its firing angle
    s << "    harmonic_cam_lobe intake_lobe(\n"
      << "        duration_at_50_thou: " << deg(camDuration) << ",\n"
      << "        gamma: 1.1,\n"
      << "        lift: " << fmt("%.0f", camLift) << " * units.thou,\n"
      << "        steps: 256\n"
      << "    )\n"
      << "    harmonic_cam_lobe exhaust_lobe(\n"
      << "        duration_at_50_thou: " << deg(camDuration + 1) << ",\n"
      << "        gamma: 1.1,\n"
      << "        lift: " << fmt("%.0f", camLift) << " * units.thou,\n"
      << "        steps: 256\n"
      << "    )\n"
      << "    camshaft_parameters cam_params(advance: 0 * units.deg, base_radius: " << fmt("%.2f", 1.0 * b) << " * units.inch)\n"
      << "    label intake_center(" << deg(lobeCenter) << ")\n"
      << "    label exhaust_center(" << deg(lobeCenter) << ")\n"
      << "    label rot360(360 * units.deg)\n";
    for (int bank = 0; bank < banks; ++bank) {
        s << "    camshaft intake_cam" << bank << "(cam_params, lobe_profile: intake_lobe)\n"
          << "    camshaft exhaust_cam" << bank << "(cam_params, lobe_profile: exhaust_lobe)\n";
        s << "    intake_cam" << bank << "\n";
        for (int i = 0; i < perBank; ++i) {
            s << "        .add_lobe(rot360 + intake_center + " << deg(spec.firing[bank * perBank + i].firingAngle) << ")\n";
        }
        s << "    exhaust_cam" << bank << "\n";
        for (int i = 0; i < perBank; ++i) {
            s << "        .add_lobe(rot360 - exhaust_center + " << deg(spec.firing[bank * perBank + i].firingAngle) << ")\n";
        }
    }
    s << "\n";

    // Header runners: each cylinder's distance to its collector, a little uneven like real ones
    // (and mirrored for the split straight six: 5, 4, 3 | 3, 4, 5 spacings)
    const auto primary = [&](int i, int bank) {
        if (splitInline) return std::to_string(i < 3 ? 5 - i : i) + " * spacing";
        return std::to_string(perBank - 1 - i) + " * spacing + " + std::to_string((i * 3 + bank * 2) % 6) + " * units.cm";
    };

    for (int bank = 0; bank < banks; ++bank) {
        s << "    b" << bank << "\n";
        for (int i = 0; i < perBank; ++i) {
            const int global = bank * perBank + i;
            s << "        .add_cylinder(\n"
              << "            piston: piston(piston_params, blowby: k_28inH2O(0.0)),\n"
              << "            connecting_rod: connecting_rod(cr_params),\n"
              << "            rod_journal: rj" << global << ",\n"
              << "            intake: intake,\n"
              << "            exhaust_system: exhaust" << (splitInline ? (i < 3 ? 0 : 1) : bank) << ",\n"
              << "            ignition_wire: wires.wire" << (global + 1) << ",\n"
              << "            sound_attenuation: 1.0,\n"
              << "            primary_length: " << primary(i, bank) << "\n"
              << "        )\n";
        }
        s << "        .set_cylinder_head(\n"
          << "            generated_head(\n"
          << "                intake_camshaft: intake_cam" << bank << ",\n"
          << "                exhaust_camshaft: exhaust_cam" << bank << ",\n"
          << "                flip_display: " << (bank == 1 ? "true" : "false") << "\n"
          << "            )\n"
          << "        )\n";
    }

    s << "\n    engine\n";
    for (int bank = 0; bank < banks; ++bank) s << "        .add_cylinder_bank(b" << bank << ")\n";
    s << "    engine.add_crankshaft(c0)\n\n";

    // Timing: the LS curve stretched over this engine's rev range
    const double stretch = spec.revLimitRpm / RevLimit;
    const double advance[] = { 12, 12, 20, 30, 40, 40, 40, 40, 40 };
    s << "    function timing_curve(" << rpm(1000 * stretch) << ")\n    timing_curve\n";
    for (int i = 0; i < 9; ++i) {
        s << "        .add_sample(" << rpm(i * 1000 * stretch) << ", " << deg(advance[i]) << ")\n";
    }
    s << "\n    ignition_module ignition_module(\n"
      << "        timing_curve: timing_curve,\n"
      << "        rev_limit: " << rpm(spec.revLimitRpm) << ",\n"
      << "        limiter_duration: " << (spec.revLimitRpm >= 8000 ? "0.1" : "0.2") << "\n"
      << "    )\n"
      << "    ignition_module\n";
    for (int i = 0; i < spec.cylinders; ++i) {
        s << "        .connect_wire(wires.wire" << (i + 1) << ", " << deg(spec.firing[i].firingAngle) << ")\n";
    }
    s << "\n    engine.add_ignition_module(ignition_module)\n"
      << "}\n\n";

    // engine-sim needs a vehicle and gearbox to run on its own; with BeamNG they're unused
    s << "private node generated_vehicle {\n"
      << "    alias output __out:\n"
      << "        vehicle(\n"
      << "            mass: 1500 * units.kg,\n"
      << "            drag_coefficient: 0.3,\n"
      << "            cross_sectional_area: (72 * units.inch) * (50 * units.inch),\n"
      << "            diff_ratio: 3.42,\n"
      << "            tire_radius: 10 * units.inch,\n"
      << "            rolling_resistance: 200 * units.N\n"
      << "        );\n"
      << "}\n\n"
      << "private node generated_transmission {\n"
      << "    alias output __out:\n"
      << "        transmission(max_clutch_torque: " << fmt("%.0f", 500 * std::max(v, 0.4)) << " * units.lb_ft)\n"
      << "        .add_gear(2.97)\n"
      << "        .add_gear(2.07)\n"
      << "        .add_gear(1.43)\n"
      << "        .add_gear(1.00)\n"
      << "        .add_gear(0.84)\n"
      << "        .add_gear(0.56);\n"
      << "}\n\n"
      << "public node main {\n"
      << "    set_engine(generated_engine())\n"
      << "    set_vehicle(generated_vehicle())\n"
      << "    set_transmission(generated_transmission())\n"
      << "}\n\n"
      << "use_default_theme()\n"
      << "main()\n";

    return s.str();
}

std::string saveGeneratedEngine(const std::string &userData, const json &vehicle, const std::string &script, std::string *error) {
    namespace fs = std::filesystem;

    // A readable, filesystem-safe name: "pickup_pickup_engine_v8_6.9"
    std::string car = read(vehicle.value("vehicle", json::object()), "directory", std::string("car"));
    while (!car.empty() && car.back() == '/') car.pop_back();
    if (car.find('/') != std::string::npos) car = car.substr(car.rfind('/') + 1);
    std::string part = read(vehicle.value("engine", json::object()), "part", std::string("engine"));
    std::string key = car + "_" + part;
    std::string clean;
    for (char ch : key) {
        if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '.' || ch == '-') clean += ch;
        else if (!clean.empty() && clean.back() != '_') clean += '_';
    }
    while (!clean.empty() && (clean.front() == '_' || clean.front() == '.')) clean.erase(clean.begin());
    if (clean.empty()) clean = "engine";

    const fs::path dir = fs::path(userData) / "generated";
    std::error_code ec;
    fs::create_directories(dir, ec);
    const fs::path path = dir / (clean + ".mr");

    std::ofstream file(path);
    file << script;
    if (!file) {
        *error = "couldn't write " + path.string();
        return {};
    }
    return path.string();
}
