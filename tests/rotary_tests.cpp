#include <gtest/gtest.h>

#include "engine_host.h"

#include "cylinder_bank.h"
#include "engine.h"
#include "simulator.h"
#include "transmission.h"
#include "units.h"

#include <cmath>
#include <filesystem>
#include <map>

namespace {
    CylinderBank rotor13b(double phase = 0) {
        CylinderBank::Parameters params{};
        params.cylinderCount = 3;
        params.rotorGeneratingRadius = units::distance(105, units::mm);
        params.rotorEccentricity = units::distance(15, units::mm);
        params.rotorWidth = units::distance(80, units::mm);
        params.rotorPhase = phase;
        CylinderBank bank;
        bank.initialize(params);
        return bank;
    }

    double deg(double d) { return d * M_PI / 180; }
}

TEST(RotaryTests, FaceGeometry) {
    const CylinderBank bank = rotor13b();
    ASSERT_TRUE(bank.isRotor());

    // A 13B face sweeps 654 cc
    const double vh = bank.rotorFaceDisplacement();
    EXPECT_NEAR(units::convert(vh, units::cc), 654.6, 0.5);

    // Smallest at compression TDC, largest 270 shaft degrees later, smallest again at overlap TDC
    EXPECT_NEAR(bank.rotorFaceSweptVolume(0, 0), 0, 1e-12);
    EXPECT_NEAR(bank.rotorFaceSweptVolume(0, deg(270)), vh, 1e-12);
    EXPECT_NEAR(bank.rotorFaceSweptVolume(0, deg(540)), 0, 1e-12);
    EXPECT_NEAR(bank.rotorFaceSweptVolume(0, deg(810)), vh, 1e-12);

    // Faces follow 360 shaft degrees apart, and between them always hold 1.5 Vh
    for (double a = 0; a < 1080; a += 7) {
        EXPECT_NEAR(bank.rotorFaceSweptVolume(1, deg(a + 360)), bank.rotorFaceSweptVolume(0, deg(a)), 1e-12);
        const double total = bank.rotorFaceSweptVolume(0, deg(a)) + bank.rotorFaceSweptVolume(1, deg(a))
            + bank.rotorFaceSweptVolume(2, deg(a));
        EXPECT_NEAR(total, 1.5 * vh, 1e-12);
    }

    // The derivative is the volume's slope (it gives the torque)
    for (double a = 3; a < 1080; a += 11) {
        const double h = 1e-6;
        const double slope = (bank.rotorFaceSweptVolume(2, deg(a) + h) - bank.rotorFaceSweptVolume(2, deg(a) - h)) / (2 * h);
        EXPECT_NEAR(bank.rotorFaceVolumeDerivative(2, deg(a)), slope, vh * 1e-6);
    }
}

namespace {
    HostPaths rotaryTestPaths(const std::filesystem::path &dir) {
        HostPaths paths;
        paths.dataRoot = ENGINE_SIM_TEST_DATA_ROOT;
        paths.userData = (dir / "user").string();
        paths.outputPath = (dir / "out").string();
        paths.configPath = (dir / "config").string();
        std::filesystem::create_directories(paths.userData);
        std::filesystem::create_directories(paths.outputPath);
        return paths;
    }
}

// The shipped 13B: 1.3 litres, makes power the right way round, and each face's ports open
// where the script says, relative to that face's own TDC
TEST(RotaryTests, Mazda13bRuns) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-rotary-test";
    std::filesystem::remove_all(dir);
    EngineHost host(rotaryTestPaths(dir));
    ASSERT_TRUE(host.loadScript(std::string(ENGINE_SIM_TEST_DATA_ROOT) + "/assets/engines/rotary/mazda_13b.mr").engineLoaded);

    auto lock = host.lock();
    Simulator *sim = host.simulator();
    Engine *engine = host.engine();
    sim->synthesizer().endAudioRenderingThread();
    EXPECT_NEAR(units::convert(engine->getDisplacement(), units::L), 1.309, 0.005);
    ASSERT_EQ(engine->getCylinderCount(), 6);

    engine->getIgnitionModule()->m_enabled = true;
    engine->setSpeedControl(1.0);
    sim->m_starterMotor.m_enabled = false;
    sim->getTransmission()->changeGear(-1);
    sim->m_dyno.m_enabled = true;
    sim->m_dyno.m_hold = true;
    sim->m_dyno.m_forwardSign = (sim->m_starterMotor.m_rotationSpeed > 0) ? 1.0 : -1.0;
    sim->m_dyno.m_rotationSpeed = units::rpm(3000);
    for (int i = 0; i < 90; ++i) { sim->startFrame(1 / 60.0); while (sim->simulateStep()) {} sim->endFrame(); }
    EXPECT_GT(units::convert(sim->getFilteredDynoTorque(), units::Nm), 50);

    // Port opening per face over one cycle
    // Centre of the time each port is over half open, as a circular mean (cycles wrap)
    struct Window { double inX = 0, inY = 0, exX = 0, exY = 0, inMax = 0, spark = -1; };
    std::map<int, Window> faces;
    // (The simulator varies its steps per frame to keep the audio in step, so count the
    // shaft's own rotation rather than time)
    for (int c = 0; c < 6; ++c) engine->getChamber(c)->popLitLastFrame();
    double turned = 0, last = engine->getOutputCrankshaft()->getCycleAngle() * 180 / M_PI;
    while (turned < 1080) {
        sim->startFrame(1 / 600.0);
        while (sim->simulateStep() && turned < 1080) {
            const double cycle = engine->getOutputCrankshaft()->getCycleAngle() * 180 / M_PI;
            turned += std::fmod(cycle - last + 1080, 1080);
            last = cycle;
            for (int c = 0; c < 6; ++c) {
                const Piston *face = engine->getPiston(c);
                CylinderHead *head = engine->getHead(face->getCylinderBank()->getIndex());
                const double in = head->intakeValveLift(face->getCylinderIndex());
                const double ex = head->exhaustValveLift(face->getCylinderIndex());
                const double phase = cycle / 1080 * 2 * M_PI;
                if (in > 0.5) { faces[c].inX += std::cos(phase); faces[c].inY += std::sin(phase); }
                if (ex > 0.5) { faces[c].exX += std::cos(phase); faces[c].exY += std::sin(phase); }
                faces[c].inMax = std::max(faces[c].inMax, in);
                if (engine->getChamber(c)->popLitLastFrame() && faces[c].spark < 0) faces[c].spark = cycle;
            }
        }
        sim->endFrame();
    }
    const double advance = engine->getIgnitionModule()->getTimingAdvance() * 180 / M_PI;

    for (int c = 0; c < 6; ++c) {
        const CylinderBank *bank = engine->getPiston(c)->getCylinderBank();
        const double tdc = bank->rotorFaceTdc(engine->getPiston(c)->getCylinderIndex()) * 180 / M_PI;
        auto wrap = [](double a) { a = std::fmod(a, 1080); return a < 0 ? a + 1080 : a; };
        // Script: intake 555-860 (centre 707.5), exhaust 195-588 (centre 391.5)
        const double intakeCentre = std::atan2(faces[c].inY, faces[c].inX) / (2 * M_PI) * 1080;
        const double exhaustCentre = std::atan2(faces[c].exY, faces[c].exX) / (2 * M_PI) * 1080;
        EXPECT_NEAR(wrap(intakeCentre - tdc), 707.5, 5) << "face " << c;
        EXPECT_NEAR(wrap(exhaustCentre - tdc), 391.5, 5) << "face " << c;
        EXPECT_NEAR(faces[c].inMax, 1.0, 0.01);
        // Each face lights its own charge, just before its own TDC
        ASSERT_GE(faces[c].spark, 0) << "face " << c << " never fired";
        EXPECT_NEAR(wrap(faces[c].spark - tdc + advance + 540) - 540, 0, 3) << "face " << c;
    }

    lock.unlock();
    std::filesystem::remove_all(dir);
}
