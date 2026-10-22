#include <gtest/gtest.h>

#include "engine_host.h"

#include "engine.h"
#include "simulator.h"
#include "transmission.h"
#include "units.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>

namespace {
    HostPaths camTestPaths(const std::filesystem::path &dir) {
        HostPaths paths;
        paths.dataRoot = ENGINE_SIM_TEST_DATA_ROOT;
        paths.userData = (dir / "user").string();
        paths.outputPath = (dir / "out").string();
        paths.configPath = (dir / "config").string();
        std::filesystem::create_directories(paths.userData);
        std::filesystem::create_directories(paths.outputPath);
        return paths;
    }

    // Spins an engine on the dyno for one cycle and reports, per cylinder, the cycle angle (deg)
    // where its spark fires and where its intake and exhaust valves are open the most
    struct Timing { double spark = -1, intakePeak = -1, exhaustPeak = -1, intakeOver50Thou = 0; };

    std::map<int, Timing> measure(const std::string &script) {
        const auto dir = std::filesystem::temp_directory_path() / "engine-sim-cam-test";
        std::filesystem::remove_all(dir);
        EngineHost host(camTestPaths(dir));
        EXPECT_TRUE(host.loadScript(script).engineLoaded);

        auto lock = host.lock();
        Simulator *sim = host.simulator();
        Engine *engine = host.engine();
        sim->synthesizer().endAudioRenderingThread();
        engine->getIgnitionModule()->m_enabled = true;
        engine->setSpeedControl(0.5);
        sim->m_starterMotor.m_enabled = false;
        sim->getTransmission()->changeGear(-1);
        sim->m_dyno.m_enabled = true;
        sim->m_dyno.m_hold = true;
        sim->m_dyno.m_forwardSign = (sim->m_starterMotor.m_rotationSpeed > 0) ? 1.0 : -1.0;
        sim->m_dyno.m_rotationSpeed = units::rpm(1500);

        // Settle, then a cycle and a bit at fine steps
        for (int i = 0; i < 60; ++i) { sim->startFrame(1 / 60.0); while (sim->simulateStep()) {} sim->endFrame(); }

        std::map<int, Timing> timing;
        std::map<int, double> bestIntake, bestExhaust;
        const int n = engine->getCylinderCount();
        IgnitionModule *ignition = engine->getIgnitionModule();
        for (int c = 0; c < n; ++c) engine->getChamber(c)->popLitLastFrame();
        sim->startFrame(0.2); // > one cycle at 1500 rpm (80 ms)
        while (sim->simulateStep()) {
            const double cycle = engine->getOutputCrankshaft()->getCycleAngle() * 180 / M_PI;
            for (int c = 0; c < n; ++c) {
                CylinderHead *head = engine->getHead(engine->getPiston(c)->getCylinderBank()->getIndex());
                const int inBank = engine->getPiston(c)->getCylinderIndex();
                const double in = head->intakeValveLift(inBank), ex = head->exhaustValveLift(inBank);
                if (in > units::distance(50, units::thou)) timing[c].intakeOver50Thou += std::abs(units::toRpm(engine->getSpeed())) * 6 * sim->getTimestep();
                if (in > bestIntake[c]) { bestIntake[c] = in; timing[c].intakePeak = cycle; }
                if (ex > bestExhaust[c]) { bestExhaust[c] = ex; timing[c].exhaustPeak = cycle; }
                if (engine->getChamber(c)->popLitLastFrame() && timing[c].spark < 0) timing[c].spark = cycle;
            }
        }
        sim->endFrame();
        lock.unlock();
        std::filesystem::remove_all(dir);
        return timing;
    }
}

double wrapDegrees(double a, double cycle) {
    a = std::fmod(a, cycle);
    return a < 0 ? a + cycle : a;
}

// Each cylinder's valves are timed to its own firing: the same spark-to-peak gap everywhere
TEST(CamTimingTests, LsValvesFollowEachCylindersFiring) {
    const auto timing = measure(std::string(ENGINE_SIM_TEST_DATA_ROOT) + "/assets/engines/atg-video-2/07_gm_ls.mr");
    ASSERT_EQ(timing.size(), 8u);
    const double intakeGap = wrapDegrees(timing.at(0).intakePeak - timing.at(0).spark, 720);
    const double exhaustGap = wrapDegrees(timing.at(0).exhaustPeak - timing.at(0).spark, 720);
    for (const auto &[c, t] : timing) {
        ASSERT_GE(t.spark, 0) << "cylinder " << c << " never fired";
        EXPECT_NEAR(wrapDegrees(t.intakePeak - t.spark, 720), intakeGap, 3) << "cylinder " << c;
        EXPECT_NEAR(wrapDegrees(t.exhaustPeak - t.spark, 720), exhaustGap, 3) << "cylinder " << c;
    }
    // Intake peaks 116 deg after overlap TDC; exhaust 116 deg before it (the LS's lobe centres)
    EXPECT_NEAR(intakeGap - exhaustGap, 232, 3);
}
