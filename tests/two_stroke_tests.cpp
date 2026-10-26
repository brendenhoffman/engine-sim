#include <gtest/gtest.h>

#include "engine_host.h"

#include "combustion_chamber.h"
#include "engine.h"
#include "simulator.h"
#include "transmission.h"
#include "units.h"

#include <algorithm>
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
}

// The KDX220: its piston pumps the crankcase above and below atmospheric every turn, its power
// valve is shut low down and open up top, and it makes real power on its pipe
TEST(TwoStrokeTests, CrankcasePumpingAndPowerValve) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-two-stroke-test";
    std::filesystem::remove_all(dir);
    EngineHost host(testPaths(dir));
    ASSERT_TRUE(host.loadScript(std::string(ENGINE_SIM_TEST_DATA_ROOT) + "/assets/engines/two-stroke/kawasaki_kdx220.mr").engineLoaded);

    auto lock = host.lock();
    Simulator *sim = host.simulator();
    Engine *engine = host.engine();
    sim->synthesizer().endAudioRenderingThread();
    engine->getIgnitionModule()->m_enabled = true;
    sim->m_starterMotor.m_enabled = false;
    sim->getTransmission()->changeGear(-1);

    CombustionChamber *chamber = engine->getChamber(0);
    ASSERT_TRUE(chamber->isTwoStroke());
    ASSERT_TRUE(chamber->hasCrankcase());

    hold(sim, engine, 4000, 1.0);
    frames(sim, 90);
    EXPECT_LT(chamber->powerValveOpening(), 0.05) << "shut at 4000 rpm";

    hold(sim, engine, 8500, 1.0);
    frames(sim, 120);
    EXPECT_GT(chamber->powerValveOpening(), 0.95) << "open at 8500 rpm";

    double low = 1e9, high = 0;
    for (int i = 0; i < 30; ++i) {
        sim->startFrame(1 / 600.0);
        while (sim->simulateStep()) {
            low = std::min(low, chamber->crankcasePressure());
            high = std::max(high, chamber->crankcasePressure());
        }
        sim->endFrame();
    }
    EXPECT_LT(low, units::pressure(1.0, units::atm));
    EXPECT_GT(high, 1.15 * units::pressure(1.0, units::atm));

    frames(sim, 60);
    EXPECT_GT(units::convert(sim->getFilteredDynoTorque(), units::Nm), 10.0);
}
