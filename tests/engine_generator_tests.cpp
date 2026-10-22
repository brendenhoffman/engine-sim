#include <gtest/gtest.h>

#include "engine_generator.h"
#include "engine_host.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>

using nlohmann::json;

namespace {
    json car(const std::string &name, const std::string &part, int cylinders = 0, json extra = json::object()) {
        json engine = { { "name", name }, { "part", part }, { "idle_rpm", 800 }, { "max_rpm", 6500 }, { "rev_limiter_rpm", 6800 } };
        if (cylinders > 0) engine["cylinders"] = cylinders;
        engine.update(extra);
        return { { "v", 1 }, { "vehicle", { { "directory", "/vehicles/test/" }, { "model", "Test Car" } } }, { "engine", engine } };
    }

    EngineSpec specFor(const json &vehicle) {
        EngineSpec spec;
        std::string error;
        EXPECT_TRUE(EngineSpec::fromVehicleInfo(vehicle, &spec, &error)) << error;
        return spec;
    }
}

TEST(EngineGeneratorTests, ReadsLayoutAndDisplacementFromNames) {
    EngineSpec v8 = specFor(car("6.9L V8 Engine", "pickup_engine_v8_6.9"));
    EXPECT_EQ(v8.layout, EngineSpec::Layout::V);
    EXPECT_EQ(v8.cylinders, 8);
    EXPECT_NEAR(v8.displacementLitres, 6.9, 1e-9);
    EXPECT_EQ(v8.bankAngle, 90);

    EngineSpec i4 = specFor(car("1.6L I4 Engine", "vivace_engine_i4_1.6_petrol"));
    EXPECT_EQ(i4.layout, EngineSpec::Layout::Inline);
    EXPECT_EQ(i4.cylinders, 4);

    EngineSpec v10 = specFor(car("", "scintilla_engine_5.0_v10"));
    EXPECT_EQ(v10.layout, EngineSpec::Layout::V);
    EXPECT_EQ(v10.cylinders, 10);
    EXPECT_NEAR(v10.displacementLitres, 5.0, 1e-9);

    EngineSpec boxer = specFor(car("2.5L Flat-4", "covet_engine_boxer"));
    EXPECT_EQ(boxer.layout, EngineSpec::Layout::Flat);
    EXPECT_EQ(boxer.cylinders, 4);
}

TEST(EngineGeneratorTests, FallsBackToSoundSamples) {
    // The Percheron's names say nothing about size; its sample does
    EngineSpec spec = specFor(car("Cherrier", "percheron_engine_I6", 6,
        { { "fuel", "diesel" }, { "exhaust_sample", "i6dt_12800cc_stock_2014_01_mn_exh" } }));
    EXPECT_EQ(spec.cylinders, 6);
    EXPECT_NEAR(spec.displacementLitres, 12.8, 1e-9);
    EXPECT_TRUE(spec.diesel);
    EXPECT_LT(spec.bore, spec.stroke); // diesels are undersquare
}

TEST(EngineGeneratorTests, NoEngineMeansNoSpec) {
    EngineSpec spec;
    std::string error;
    EXPECT_FALSE(EngineSpec::fromVehicleInfo(json{ { "vehicle", { { "directory", "electric" } } } }, &spec, &error));
    EXPECT_FALSE(error.empty());
}

TEST(EngineGeneratorTests, DisplacementMatchesBoreAndStroke) {
    const EngineSpec spec = specFor(car("3.3L V6 Engine", "legran_engine_v6_3.3"));
    const double perCylinder = 3.14159265358979 / 4 * spec.bore * spec.bore * spec.stroke;
    EXPECT_NEAR(perCylinder * spec.cylinders * 1000, 3.3, 1e-6);
}

TEST(EngineGeneratorTests, EveryCylinderFiresOnceAtEvenIntervals) {
    const struct { const char *name; const char *part; } engines[] = {
        { "1.2L I3", "a_i3" }, { "2.0L I4", "a_i4" }, { "2.5L I5", "a_i5" }, { "3.0L I6", "a_i6" },
        { "3.5L V6", "a_v6" }, { "5.0L V8", "a_v8" }, { "5.0L V10", "a_v10" }, { "6.0L V12", "a_v12" },
        { "2.5L Flat-4", "a_h4" }, { "3.6L Flat-6", "a_h6" },
    };
    for (const auto &e : engines) {
        const EngineSpec spec = specFor(car(e.name, e.part));
        ASSERT_EQ(static_cast<int>(spec.firing.size()), spec.cylinders) << e.name;

        std::set<long> angles;
        for (const auto &cyl : spec.firing) angles.insert(std::lround(cyl.firingAngle * 1000));
        EXPECT_EQ(static_cast<int>(angles.size()), spec.cylinders) << e.name << ": two cylinders fire together";

        const double interval = 720.0 / spec.cylinders;
        for (long a : angles) EXPECT_NEAR(std::fmod(a / 1000.0, interval), 0.0, 1e-6) << e.name;
    }
}

// The generated scripts must compile and create an engine for every layout
TEST(EngineGeneratorTests, GeneratedScriptsLoad) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-generator-test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);

    HostPaths paths;
    paths.dataRoot = ENGINE_SIM_TEST_DATA_ROOT;
    paths.userData = (dir / "user").string();
    paths.outputPath = (dir / "out").string();
    paths.configPath = (dir / "config").string();
    std::filesystem::create_directories(paths.userData);
    std::filesystem::create_directories(paths.outputPath);

    const char *names[] = { "1.2L I3", "2.0L I4", "3.0L I6", "3.5L V6", "5.0L V8", "5.0L V10", "6.0L V12", "2.5L Flat-4" };
    for (const char *name : names) {
        const EngineSpec spec = specFor(car(name, "x"));
        const std::string path = (dir / (spec.layoutName() + ".mr")).string();
        std::ofstream(path) << generateEngineScript(spec);

        EngineHost host(paths);
        const EngineHost::LoadResult result = host.loadScript(path);
        EXPECT_TRUE(result.replaced && !result.usedFallback) << name << ": " << result.message << "\n" << result.errors;
    }

    std::filesystem::remove_all(dir);
}
