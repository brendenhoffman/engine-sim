#include <gtest/gtest.h>

#include "engine_host.h"

#include "combustion_chamber.h"
#include "engine.h"
#include "forced_induction.h"
#include "intake.h"
#include "simulator.h"
#include "transmission.h"
#include "units.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <vector>

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

    void hold(Simulator *sim, Engine *engine, double rpm, double throttle) {
        engine->setSpeedControl(throttle);
        sim->m_dyno.m_enabled = true;
        sim->m_dyno.m_hold = true;
        sim->m_dyno.m_forwardSign = (sim->m_starterMotor.m_rotationSpeed > 0) ? 1.0 : -1.0;
        sim->m_dyno.m_rotationSpeed = units::rpm(rpm);
    }

    void frames(Simulator *sim, int n) {
        for (int i = 0; i < n; ++i) {
            sim->startFrame(1 / 60.0);
            while (sim->simulateStep()) {}
            sim->endFrame();
        }
    }

    void start(Simulator *sim, Engine *engine) {
        sim->synthesizer().endAudioRenderingThread();
        engine->getIgnitionModule()->m_enabled = true;
        sim->m_starterMotor.m_enabled = false;
        sim->getTransmission()->changeGear(-1);
    }

    // Each cylinder's highest pressure over a few turns (0.1 s)
    std::vector<double> peakPressures(Simulator *sim, Engine *engine) {
        std::vector<double> peak(engine->getCylinderCount(), 0.0);
        for (int i = 0; i < 60; ++i) {
            sim->startFrame(1 / 600.0);
            while (sim->simulateStep()) {
                for (int j = 0; j < engine->getCylinderCount(); ++j) {
                    peak[j] = std::max(peak[j], engine->getChamber(j)->m_system.pressure());
                }
            }
            sim->endFrame();
        }
        return peak;
    }
}

// The 8V-71N: a blower-scavenged two-stroke with exhaust valves in the head. Every cylinder in
// both banks fires (the injectors and cams are timed to each bank's TDC), it makes about its
// rated torque, and the governor holds its idle.
TEST(DetroitTests, BlowerTwoStrokeFiresEveryCylinder) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-detroit-test";
    std::filesystem::remove_all(dir);
    EngineHost host(testPaths(dir));
    ASSERT_TRUE(host.loadScript(std::string(ENGINE_SIM_TEST_DATA_ROOT) + "/assets/engines/detroit/8v71n.mr").engineLoaded);

    auto lock = host.lock();
    Simulator *sim = host.simulator();
    Engine *engine = host.engine();
    start(sim, engine);
    ASSERT_EQ(engine->getCylinderCount(), 8);
    ASSERT_TRUE(engine->getChamber(0)->isTwoStroke());
    ASSERT_FALSE(engine->getChamber(0)->hasCrankcase());

    hold(sim, engine, 1800, 1.0);
    frames(sim, 120);
    const double torque = units::convert(sim->getFilteredDynoTorque(), units::Nm);
    EXPECT_GT(torque, 1000);
    EXPECT_LT(torque, 1350);
    for (double p : peakPressures(sim, engine)) {
        EXPECT_GT(p, units::pressure(70, units::atm)) << "every cylinder lights";
    }
    const ForcedInduction *fi = engine->getIntake(0)->getForcedInduction();
    ASSERT_NE(fi, nullptr);
    EXPECT_GT(fi->boost(), 5e3) << "the blower fills the airbox";
    EXPECT_EQ(fi->interstage(), nullptr);

    sim->m_dyno.m_enabled = false;
    engine->setSpeedControl(0.0);
    frames(sim, 360);
    EXPECT_NEAR(engine->getRpm(), 550, 60);
}

// The 6V-92TA: its turbo feeds the blower, so the airbox sits above what the turbo delivers,
// which sits above the open air
TEST(DetroitTests, TurboFeedsBlower) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-detroit-test-ta";
    std::filesystem::remove_all(dir);
    EngineHost host(testPaths(dir));
    ASSERT_TRUE(host.loadScript(std::string(ENGINE_SIM_TEST_DATA_ROOT) + "/assets/engines/detroit/6v92ta.mr").engineLoaded);

    auto lock = host.lock();
    Simulator *sim = host.simulator();
    Engine *engine = host.engine();
    start(sim, engine);
    const ForcedInduction *fi = engine->getIntake(0)->getForcedInduction();
    ASSERT_NE(fi, nullptr);
    ASSERT_EQ(fi->stageCount(), 2);
    ASSERT_NE(fi->interstage(), nullptr);

    hold(sim, engine, 1800, 1.0);
    frames(sim, 240);
    const double turbo = fi->interstage()->pressure() - ForcedInduction::Ambient;
    EXPECT_GT(turbo, 30e3) << "the turbo spools";
    EXPECT_GT(fi->boost(), turbo + 5e3) << "the blower adds to it";
    EXPECT_GT(fi->state(1).massFlow, 0.3);
    EXPECT_NEAR(fi->state(0).massFlow, fi->state(1).massFlow, 0.1 * fi->state(1).massFlow)
        << "what the turbo delivers, the blower takes";
    EXPECT_GT(units::convert(sim->getFilteredDynoTorque(), units::Nm), 1150);
    for (double p : peakPressures(sim, engine)) {
        EXPECT_GT(p, units::pressure(90, units::atm));
    }
}
