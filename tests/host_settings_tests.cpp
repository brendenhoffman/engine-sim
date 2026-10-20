#include <gtest/gtest.h>

#include "host_settings.h"

#include <filesystem>
#include <fstream>

using nlohmann::json;

TEST(HostSettingsTests, Defaults) {
    const HostSettings settings;
    EXPECT_TRUE(settings.bridge.connect);
    EXPECT_FLOAT_EQ(settings.bridge.volume, 0.20f);
    EXPECT_EQ(settings.headroomDb, 6);
    EXPECT_FLOAT_EQ(settings.bridge.beamIdleRpm, 1100.0f);
    EXPECT_FALSE(settings.bridge.tuneBeamEngine);
    EXPECT_TRUE(settings.bridge.neutralEq);
    EXPECT_TRUE(settings.bridge.intakeSound);
    EXPECT_TRUE(settings.bridge.exhaustSound);
    EXPECT_EQ(settings.simulationFrequency, 0);
    EXPECT_TRUE(settings.script.empty());
}

TEST(HostSettingsTests, MergeIsPartial) {
    HostSettings settings;
    std::string error;
    ASSERT_TRUE(settings.merge(json{ { "bridge", { { "volume", 0.5 } } } }, &error)) << error;

    EXPECT_FLOAT_EQ(settings.bridge.volume, 0.5f);
    EXPECT_TRUE(settings.bridge.connect);
    EXPECT_FLOAT_EQ(settings.bridge.beamIdleRpm, 1100.0f);
}

TEST(HostSettingsTests, InvalidMergeChangesNothing) {
    HostSettings settings;
    std::string error;

    // The valid field must not be applied when another one is bad
    EXPECT_FALSE(settings.merge(json{ { "simulation_frequency", 20000 }, { "bridge", { { "volume", "loud" } } } }, &error));
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(settings.simulationFrequency, 0);
    EXPECT_FLOAT_EQ(settings.bridge.volume, 0.20f);

    EXPECT_FALSE(settings.merge(json{ { "simulation_frequency", 100 } }, &error));
    EXPECT_FALSE(settings.merge(json{ { "bridge", { { "volume", -1 } } } }, &error));
    EXPECT_FALSE(settings.merge(json{ { "bridge", true } }, &error));
    EXPECT_FALSE(settings.merge(json::array(), &error));
}

TEST(HostSettingsTests, UnknownKeysAreIgnored) {
    HostSettings settings;
    std::string error;
    EXPECT_TRUE(settings.merge(json{ { "from_the_future", 1 }, { "bridge", { { "also_new", true } } } }, &error)) << error;
}

TEST(HostSettingsTests, SaveAndLoadRoundTrip) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-settings-test";
    std::filesystem::remove_all(dir);
    const std::string path = (dir / "settings.json").string();

    HostSettings settings;
    settings.script = "/somewhere/main.mr";
    settings.simulationFrequency = 12000;
    settings.bridge.tuneBeamEngine = true;
    settings.bridge.neutralEq = false;
    settings.bridge.intakeSound = false;

    std::string error;
    ASSERT_TRUE(settings.save(path, &error)) << error;

    const HostSettings loaded = HostSettings::load(path, &error);
    EXPECT_TRUE(error.empty()) << error;
    EXPECT_EQ(loaded.script, settings.script);
    EXPECT_EQ(loaded.simulationFrequency, 12000);
    EXPECT_TRUE(loaded.bridge.tuneBeamEngine);
    EXPECT_FALSE(loaded.bridge.neutralEq);
    EXPECT_FALSE(loaded.bridge.intakeSound);
    EXPECT_TRUE(loaded.bridge.exhaustSound);

    std::filesystem::remove_all(dir);
}

TEST(HostSettingsTests, CorruptFileFallsBackToDefaults) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-settings-test-corrupt";
    std::filesystem::create_directories(dir);
    const std::string path = (dir / "settings.json").string();
    std::ofstream(path) << "{ not json";

    std::string error;
    const HostSettings loaded = HostSettings::load(path, &error);
    EXPECT_FALSE(error.empty());
    EXPECT_FLOAT_EQ(loaded.bridge.volume, 0.20f);

    std::filesystem::remove_all(dir);
}

TEST(HostSettingsTests, MissingFileIsNotAnError) {
    std::string error;
    HostSettings::load("/nonexistent/engine-sim/settings.json", &error);
    EXPECT_TRUE(error.empty());
}
