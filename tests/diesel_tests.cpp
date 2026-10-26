#include <gtest/gtest.h>

#include "engine_host.h"

#include "engine.h"
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

    void hold(Simulator *sim, Engine *engine, double rpm, double pedal) {
        engine->setSpeedControl(pedal);
        sim->m_dyno.m_enabled = true;
        sim->m_dyno.m_hold = true;
        sim->m_dyno.m_forwardSign = (sim->m_starterMotor.m_rotationSpeed > 0) ? 1.0 : -1.0;
        sim->m_dyno.m_rotationSpeed = units::rpm(rpm);
    }

    double torque(Simulator *sim) { return units::convert(sim->getFilteredDynoTorque(), units::Nm); }
}

// The Cummins: plain air in the intake, real torque under the pump's full delivery, its
// governor cutting the fuel above rated speed, and the Jake brake pulling harder off throttle
TEST(DieselTests, CumminsGovernorAndJake) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-diesel-test";
    std::filesystem::remove_all(dir);
    EngineHost host(testPaths(dir));
    ASSERT_TRUE(host.loadScript(std::string(ENGINE_SIM_TEST_DATA_ROOT) + "/assets/engines/cummins/6bt.mr").engineLoaded);

    auto lock = host.lock();
    Simulator *sim = host.simulator();
    Engine *engine = host.engine();
    sim->synthesizer().endAudioRenderingThread();
    engine->getIgnitionModule()->m_enabled = true;
    sim->m_starterMotor.m_enabled = false;
    sim->getTransmission()->changeGear(-1);

    ASSERT_TRUE(engine->getFuel()->isCompressionIgnition());
    EXPECT_EQ(engine->getIntake(0)->freshMix().p_fuel, 0.0);

    hold(sim, engine, 2000, 1.0);
    frames(sim, 300);
    EXPECT_NEAR(engine->getDieselFueling(), 1.0, 1e-9);
    double average = 0;
    for (int i = 0; i < 60; ++i) { frames(sim, 1); average += torque(sim) / 60; }
    EXPECT_GT(average, 500);
    EXPECT_LT(average, 750);
    EXPECT_LT(engine->getIntake(0)->m_system.n_fuel(), 1e-6) << "no fuel in the intake";

    hold(sim, engine, 2750, 1.0);
    frames(sim, 120);
    EXPECT_LT(engine->getDieselFueling(), 0.1) << "the governor cuts the fuel above rated speed";

    engine->setJakeBrakeEnabled(false);
    hold(sim, engine, 2200, 0.0);
    frames(sim, 120);
    const double coasting = torque(sim);
    engine->setJakeBrakeEnabled(true);
    frames(sim, 120);
    EXPECT_TRUE(engine->isJakeBrakeActive());
    EXPECT_LT(torque(sim), coasting - 100) << "the Jake brake pulls harder";
}

// Off the dyno, the governor holds the idle
TEST(DieselTests, GovernorHoldsIdle) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-diesel-idle-test";
    std::filesystem::remove_all(dir);
    EngineHost host(testPaths(dir));
    ASSERT_TRUE(host.loadScript(std::string(ENGINE_SIM_TEST_DATA_ROOT) + "/assets/engines/cummins/6bt.mr").engineLoaded);

    auto lock = host.lock();
    Simulator *sim = host.simulator();
    Engine *engine = host.engine();
    sim->synthesizer().endAudioRenderingThread();
    engine->getIgnitionModule()->m_enabled = true;
    sim->m_starterMotor.m_enabled = false;
    sim->getTransmission()->changeGear(-1);

    hold(sim, engine, 1200, 0.0);
    frames(sim, 60);
    engine->setSpeedControl(0.0);
    sim->m_dyno.m_enabled = false;
    sim->m_dyno.m_hold = false;
    frames(sim, 300);
    EXPECT_NEAR(std::abs(engine->getRpm()), 800, 60);
}
