#include <gtest/gtest.h>

#include "engine_host.h"

#include "engine.h"
#include "forced_induction.h"
#include "intake.h"
#include "simulator.h"
#include "transmission.h"
#include "units.h"

#include <cmath>
#include <filesystem>

namespace {
    HostPaths testPaths(const std::filesystem::path &dir) {
        HostPaths paths;
        paths.dataRoot = ENGINE_SIM_TEST_DATA_ROOT;
        paths.userData = (dir / "user").string();
        paths.outputPath = (dir / "out").string();
        paths.configPath = (dir / "config").string();
        std::filesystem::create_directories(paths.userData);
        std::filesystem::create_directories(paths.outputPath);
        return paths;
    }

    void frames(Simulator *sim, int n) {
        for (int i = 0; i < n; ++i) {
            sim->startFrame(1 / 60.0);
            while (sim->simulateStep()) {}
            sim->endFrame();
        }
    }

    void hold(Simulator *sim, Engine *engine, double rpm, double throttle) {
        engine->setSpeedControl(throttle);
        sim->m_dyno.m_enabled = true;
        sim->m_dyno.m_hold = true;
        sim->m_dyno.m_forwardSign = (sim->m_starterMotor.m_rotationSpeed > 0) ? 1.0 : -1.0;
        sim->m_dyno.m_rotationSpeed = units::rpm(rpm);
    }

    void start(Simulator *sim, Engine *engine) {
        sim->synthesizer().endAudioRenderingThread();
        engine->getIgnitionModule()->m_enabled = true;
        sim->m_starterMotor.m_enabled = false;
        sim->getTransmission()->changeGear(-1);
    }
}

// The compressor characteristic: most of its pressure at no flow, a peak at the surge line,
// nothing left at choke, and all of it scaling with tip speed squared
TEST(ForcedInductionTests, CompressorCharacteristic) {
    ForcedInduction fi;
    ForcedInduction::Stage s;
    s.compressorRadius = 0.025;
    s.compressorFlow = 0.25;
    s.maxSpeed = 2e4;
    const double speed = 1.5e4;
    const double choke = s.compressorFlow * speed / s.maxSpeed;
    const double shutOff = fi.compressorRise(s, speed, 0, nullptr);
    const double peak = fi.compressorRise(s, speed, choke / 2, nullptr);
    EXPECT_NEAR(shutOff / peak, 0.8, 1e-9);
    EXPECT_GT(peak, fi.compressorRise(s, speed, 0.4 * choke, nullptr));
    EXPECT_GT(peak, fi.compressorRise(s, speed, 0.6 * choke, nullptr));
    EXPECT_NEAR(fi.compressorRise(s, speed, choke, nullptr) / peak, 0, 1e-9);
    // Rising flow left of the peak: the unstable side, where surge lives
    EXPECT_LT(fi.compressorRise(s, speed, 0.2 * choke, nullptr), fi.compressorRise(s, speed, 0.4 * choke, nullptr));
    // Tip speed squared, roughly (the pressure ratio is a power of it)
    const double half = fi.compressorRise(s, speed / 2, choke / 4, nullptr);
    EXPECT_GT(peak / half, 3.5);
    EXPECT_LT(peak / half, 5.5);
}

// The 13B-REW spools up under load, holds about its wastegate's boost, and blows off when the
// throttle shuts
TEST(ForcedInductionTests, TurboSpoolsAndBlowsOff) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-fi-test";
    std::filesystem::remove_all(dir);
    EngineHost host(testPaths(dir));
    ASSERT_TRUE(host.loadScript(std::string(ENGINE_SIM_TEST_DATA_ROOT) + "/assets/engines/rotary/mazda_13b_rew.mr").engineLoaded);

    auto lock = host.lock();
    Simulator *sim = host.simulator();
    Engine *engine = host.engine();
    start(sim, engine);
    ASSERT_EQ(engine->getCompressorStageCount(), 2);
    const ForcedInduction *fi = engine->getIntake(0)->getForcedInduction();
    ASSERT_NE(fi, nullptr);

    hold(sim, engine, 6000, 1.0);
    frames(sim, 240);
    const double boost = fi->boost();
    EXPECT_GT(boost, 40e3);
    EXPECT_LT(boost, 90e3);
    EXPECT_TRUE(fi->state(1).engaged);
    EXPECT_GT(fi->state(1).massFlow, 0.02) << "both turbos flow above the changeover";
    EXPECT_GT(units::convert(sim->getFilteredDynoTorque(), units::Nm), 230);

    hold(sim, engine, 6000, 0.0);
    double bov = 0;
    for (int i = 0; i < 20; ++i) { frames(sim, 1); bov = std::max(bov, fi->voice(0).bov); }
    EXPECT_GT(bov, 0.1) << "the blow-off valve vents";
    frames(sim, 60);
    EXPECT_LT(fi->boost(), 0.3 * boost);
}

// The H2's supercharger is driven by the crank and, with no blow-off valve, surges when the
// throttle shuts at speed: its flow keeps reversing
TEST(ForcedInductionTests, SuperchargerSurges) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-fi-test-h2";
    std::filesystem::remove_all(dir);
    EngineHost host(testPaths(dir));
    ASSERT_TRUE(host.loadScript(std::string(ENGINE_SIM_TEST_DATA_ROOT) + "/assets/engines/kawasaki/ninja_h2.mr").engineLoaded);

    auto lock = host.lock();
    Simulator *sim = host.simulator();
    Engine *engine = host.engine();
    start(sim, engine);
    const ForcedInduction *fi = engine->getIntake(0)->getForcedInduction();
    ASSERT_NE(fi, nullptr);

    hold(sim, engine, 10000, 1.0);
    frames(sim, 120);
    EXPECT_NEAR(fi->state(0).speed / std::abs(engine->getSpeed()), 9.18, 0.01);
    EXPECT_GT(fi->boost(), 30e3);
    EXPECT_GT(fi->crankLoad(), 2.0) << "the crank pays for the boost";

    hold(sim, engine, 10000, 0.0);
    int reversals = 0;
    bool backwards = false;
    for (int i = 0; i < 60; ++i) {
        frames(sim, 1);
        const bool now = fi->state(0).massFlow < 0;
        if (now && !backwards) ++reversals;
        backwards = now;
    }
    EXPECT_GT(reversals, 5);
}
