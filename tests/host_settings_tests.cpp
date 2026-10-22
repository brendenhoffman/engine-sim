#include <gtest/gtest.h>

#include "host_settings.h"

#include <filesystem>
#include <fstream>

using nlohmann::json;

TEST(HostSettingsTests, Defaults) {
    const HostSettings settings;
    EXPECT_TRUE(settings.bridge.connect);
    EXPECT_FALSE(settings.bridge.driveCar);
    EXPECT_TRUE(settings.bridge.matchCarPower);
    EXPECT_FLOAT_EQ(settings.bridge.volume, 0.20f);
    EXPECT_EQ(settings.headroomDb, 6);
    EXPECT_FLOAT_EQ(settings.bridge.beamIdleRpm, 1100.0f);
    EXPECT_FALSE(settings.bridge.tuneBeamEngine);
    EXPECT_TRUE(settings.bridge.neutralEq);
    EXPECT_FLOAT_EQ(settings.bridge.intakeLevel, 0.35f);
    EXPECT_FLOAT_EQ(settings.bridge.exhaustLevel, 1.0f);
    EXPECT_FALSE(settings.bridge.keepGameLowCut);
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
    settings.bridge.intakeLevel = 0.5f;

    std::string error;
    ASSERT_TRUE(settings.save(path, &error)) << error;

    const HostSettings loaded = HostSettings::load(path, &error);
    EXPECT_TRUE(error.empty()) << error;
    EXPECT_EQ(loaded.script, settings.script);
    EXPECT_EQ(loaded.simulationFrequency, 12000);
    EXPECT_TRUE(loaded.bridge.tuneBeamEngine);
    EXPECT_FALSE(loaded.bridge.neutralEq);
    EXPECT_FLOAT_EQ(loaded.bridge.intakeLevel, 0.5f);
    EXPECT_FLOAT_EQ(loaded.bridge.exhaustLevel, 1.0f);

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

TEST(HostSettingsTests, OldOnOffSwitchesBecomeLevels) {
    HostSettings settings;
    std::string error;
    ASSERT_TRUE(settings.merge(json{ { "bridge", { { "intake_sound", false }, { "exhaust_sound", true } } } }, &error)) << error;
    EXPECT_FLOAT_EQ(settings.bridge.intakeLevel, 0.0f);
    EXPECT_FLOAT_EQ(settings.bridge.exhaustLevel, 1.0f);
}

TEST(HostSettingsTests, MissingFileIsNotAnError) {
    std::string error;
    HostSettings::load("/nonexistent/engine-sim/settings.json", &error);
    EXPECT_TRUE(error.empty());
}

// Settings saved before the level memory default changed carry the old default explicitly;
// they get the new default, while a value someone chose is kept
TEST(HostSettingsTests, OldLevelMemoryDefaultIsUpgraded) {
    HostSettings settings;
    std::string error;
    ASSERT_TRUE(settings.merge(nlohmann::json{ { "audio", { { "leveler_release", 0.02266 } } } }, &error)) << error;
    EXPECT_DOUBLE_EQ(settings.levelerRelease, HostSettings().levelerRelease);
    ASSERT_TRUE(settings.merge(nlohmann::json{ { "audio", { { "leveler_release", 2.0 } } } }, &error)) << error;
    EXPECT_DOUBLE_EQ(settings.levelerRelease, HostSettings().levelerRelease);
    ASSERT_TRUE(settings.merge(nlohmann::json{ { "audio", { { "leveler_release", 0.5 } } } }, &error)) << error;
    EXPECT_DOUBLE_EQ(settings.levelerRelease, 0.5);
}
