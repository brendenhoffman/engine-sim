// engine-sim-bench: runs engines as fast as the machine allows, without real-time pacing or the
// audio thread, so the same engine gives the same numbers every run. For checking that changes to
// the simulation leave existing engines alone, and for rendering repeatable audio to listen to.
//
//   engine-sim-bench data/assets/engines/atg-video-2/07_gm_ls.mr
//   engine-sim-bench --json now.json --compare before.json ENGINE...
//   engine-sim-bench --wav renders/ ENGINE...

#include "engine_host.h"

#include "engine.h"
#include "forced_induction.h"
#include "exhaust_system.h"
#include "intake.h"
#include "ignition_module.h"
#include "simulator.h"
#include "synthesizer.h"
#include "transmission.h"
#include "crankshaft.h"
#include "combustion_chamber.h"
#include "cylinder_bank.h"
#include "constants.h"
#include "units.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using nlohmann::json;

namespace {
    struct Options {
        std::vector<std::string> scripts;
        std::string jsonOut;
        std::string compareWith;
        std::string wavDir;
        double tolerance = 2.0; // percent of peak torque
        int points = 10;
        bool dyno = true;
        bool traceIdle = false;
        bool traceCombustion = false;
        double levelerRelease = 0; // 0: the default
        double idleRamp = 1.0;     // Seconds to bring it down from the rev limit before idling
        double idleStart = 1000;   // Where it's let go to find its idle
        double holdIdle = 0;
        double traceBurn = 0;      // Report how full-throttle burns go at this rpm
        double liftRpm = 0;        // Throttle snapped shut on boost at this speed (blow-off, surge)
        double steadyRpm = 0;      // With --wav: render a steady hold instead of the run-up
        double steadyThrottle = 0;
        bool startStop = false;
        bool closedCurve = false;  // Also measure closed-throttle torque at every dyno point
        double coupleRpm = 0, coupleHz = 0; // Idle loosely held as the BeamNG bridge does
        bool coupleIac = true;
        bool coupleFree = false;      // Like BeamBridge's free-running idle: hands off within 300 rpm
        double coupleSeconds = 20;    // With --wav: render a start from rest, idle, and switch-off       // Hold idle at this speed (as BeamNG does) instead of letting it find its own
    };

    void usage() {
        std::fprintf(stderr,
            "usage: engine-sim-bench [options] ENGINE.mr...\n"
            "  --json FILE       write the results as JSON\n"
            "  --compare FILE    compare with earlier --json results; exit 1 if anything moved\n"
            "  --tolerance PCT   allowed torque change, percent of peak torque (default 2)\n"
            "  --points N        dyno points from 1000 rpm to the rev limit (default 10)\n"
            "  --wav DIR         also render a run-up for each engine to DIR/<name>.wav\n"
            "  --no-dyno         only render (with --wav)\n"
            "  --leveler-release S  how long the output leveler holds a peak (default upstream's 0.023)\n"
            "  --trace-burn RPM  how full-throttle burns go at this rpm (completion, flame speed, timing)\n"
            "  --steady RPM[:THROTTLE]  with --wav, render 4 s held at this rpm and throttle instead\n"
            "  --couple RPM:HZ   20 s of idle loosely held at RPM as the BeamNG bridge does (PI at HZ), in 5 s windows\n"
            "  --couple-free     ...hands off within 300 rpm of it (BeamNG's free-running idle)\n"
            "  --couple-no-iac   ...with the engine's own idle control off\n"
            "  --closed          also measure closed-throttle torque at each dyno point\n"
            "  --lift RPM        held at RPM: 3 s full throttle, 2 s shut, 1.5 s full again; traces the\n"
            "                    forced induction every 20 ms (renders it with --wav)\n"
            "  --start-stop      with --wav, render cranking from rest, 3 s idle, then ignition off\n"
            "  --hold-idle RPM   idle held at this speed with the throttle shut, as in BeamNG\n"
            "  --idle-start RPM  where the idle test lets go (default 1000)\n"
            "  --idle-ramp S     seconds to come down from the limit before the idle test (default 1)\n"
            "  --trace-combustion  print every firing while idling (residual gas, mixture, burn)\n"
            "  --trace-idle      print the rpm every 0.1 s while settling to idle\n");
    }

    bool parse(int argc, char **argv, Options *options) {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto value = [&](const char *name) -> const char * {
                if (i + 1 >= argc) {
                    std::fprintf(stderr, "engine-sim-bench: %s needs a value\n", name);
                    return nullptr;
                }
                return argv[++i];
            };

            if (arg == "--json") { const char *v = value("--json"); if (!v) return false; options->jsonOut = v; }
            else if (arg == "--compare") { const char *v = value("--compare"); if (!v) return false; options->compareWith = v; }
            else if (arg == "--lift") { const char *v = value("--lift"); if (!v) return false; options->liftRpm = std::atof(v); }
            else if (arg == "--wav") { const char *v = value("--wav"); if (!v) return false; options->wavDir = v; }
            else if (arg == "--tolerance") { const char *v = value("--tolerance"); if (!v) return false; options->tolerance = std::atof(v); }
            else if (arg == "--points") { const char *v = value("--points"); if (!v) return false; options->points = std::max(2, std::atoi(v)); }
            else if (arg == "--no-dyno") options->dyno = false;
            else if (arg == "--trace-idle") options->traceIdle = true;
            else if (arg == "--trace-combustion") options->traceCombustion = true;
            else if (arg == "--trace-burn") { const char *v = value("--trace-burn"); if (!v) return false; options->traceBurn = std::atof(v); }
            else if (arg == "--steady") {
                const char *v = value("--steady"); if (!v) return false;
                if (std::sscanf(v, "%lf:%lf", &options->steadyRpm, &options->steadyThrottle) < 1) return false;
            }
            else if (arg == "--start-stop") options->startStop = true;
            else if (arg == "--closed") options->closedCurve = true;
            else if (arg == "--couple") {
                const char *v = value("--couple"); if (!v) return false;
                if (std::sscanf(v, "%lf:%lf", &options->coupleRpm, &options->coupleHz) != 2) return false;
            }
            else if (arg == "--couple-no-iac") options->coupleIac = false;
            else if (arg == "--couple-free") options->coupleFree = true;
            else if (arg == "--couple-seconds") { const char *v = value("--couple-seconds"); if (!v) return false; options->coupleSeconds = std::atof(v); }
            else if (arg == "--hold-idle") { const char *v = value("--hold-idle"); if (!v) return false; options->holdIdle = std::atof(v); }
            else if (arg == "--idle-start") { const char *v = value("--idle-start"); if (!v) return false; options->idleStart = std::atof(v); }
            else if (arg == "--idle-ramp") { const char *v = value("--idle-ramp"); if (!v) return false; options->idleRamp = std::atof(v); }
            else if (arg == "--leveler-release") { const char *v = value("--leveler-release"); if (!v) return false; options->levelerRelease = std::atof(v); }
            else if (arg == "-h" || arg == "--help") { usage(); std::exit(0); }
            else if (!arg.empty() && arg[0] == '-') { std::fprintf(stderr, "engine-sim-bench: unknown option %s\n", arg.c_str()); return false; }
            else options->scripts.push_back(arg);
        }
        if (options->scripts.empty()) {
            usage();
            return false;
        }
        return true;
    }

    // Steps the simulation and renders its audio on this thread
    class Bench {
    public:
        Bench(Simulator *simulator, Engine *engine) : m_simulator(simulator), m_engine(engine) {
            // Render on this thread, in step with the simulation
            m_simulator->synthesizer().endAudioRenderingThread();
            std::srand(1);
        }

        void frame(double dt = 1 / 60.0) {
            m_simulator->startFrame(dt);
            while (m_simulator->simulateStep()) {
                if (m_stepHook) m_stepHook();
            }
            m_simulator->endFrame();
            m_simulator->synthesizer().renderAudio();

            int16_t buffer[4096];
            const int n = m_simulator->readAudioOutput(4096, buffer);
            if (m_recording) m_audio.insert(m_audio.end(), buffer, buffer + n);
            m_lastAudio.assign(buffer, buffer + n);
        }

        void run(double seconds, const std::function<void()> &eachFrame = nullptr) {
            const int frames = static_cast<int>(std::round(seconds * 60));
            for (int i = 0; i < frames; ++i) {
                frame();
                if (eachFrame) eachFrame();
            }
        }

        void hold(double rpm, double throttle) {
            m_engine->setSpeedControl(throttle);
            m_simulator->m_dyno.m_enabled = true;
            m_simulator->m_dyno.m_hold = true;
            m_simulator->m_dyno.m_forwardSign = (m_simulator->m_starterMotor.m_rotationSpeed > 0) ? 1.0 : -1.0;
            m_simulator->m_dyno.m_rotationSpeed = units::rpm(rpm);
        }

        void release(double throttle) {
            m_engine->setSpeedControl(throttle);
            m_simulator->m_dyno.m_enabled = false;
            m_simulator->m_dyno.m_hold = false;
        }

        void record(bool on) { m_recording = on; }
        void setStepHook(std::function<void()> hook) { m_stepHook = std::move(hook); }
        const std::vector<int16_t> &audio() const { return m_audio; }
        const std::vector<int16_t> &lastAudio() const { return m_lastAudio; }

    private:
        Simulator *m_simulator;
        Engine *m_engine;
        bool m_recording = false;
        std::function<void()> m_stepHook;
        std::vector<int16_t> m_audio, m_lastAudio;
    };

    // Loudness and how much of it is high-frequency (first-difference energy over total), from
    // the synthesizer output with its leveler gain divided back out
    struct SoundStats {
        double sumSquares = 0, diffSquares = 0;
        long count = 0;
        int16_t last = 0;

        void add(const std::vector<int16_t> &samples, double levelerGain) {
            const double g = levelerGain > 1e-6 ? 1 / levelerGain : 1;
            for (int16_t s : samples) {
                const double x = s * g, d = (s - last) * g;
                sumSquares += x * x;
                diffSquares += d * d;
                last = s;
                ++count;
            }
        }
        double rms() const { return count ? std::sqrt(sumSquares / count) : 0; }
        double brightness() const { return sumSquares > 0 ? std::sqrt(diffSquares / sumSquares) : 0; }
    };

    void writeWav(const std::string &path, const std::vector<int16_t> &samples, int rate) {
        std::ofstream out(path, std::ios::binary);
        auto u32 = [&](uint32_t v) { out.write(reinterpret_cast<const char *>(&v), 4); };
        auto u16 = [&](uint16_t v) { out.write(reinterpret_cast<const char *>(&v), 2); };
        const uint32_t bytes = static_cast<uint32_t>(samples.size() * 2);
        out.write("RIFF", 4); u32(36 + bytes); out.write("WAVE", 4);
        out.write("fmt ", 4); u32(16); u16(1); u16(1); u32(rate); u32(rate * 2); u16(2); u16(16);
        out.write("data", 4); u32(bytes);
        out.write(reinterpret_cast<const char *>(samples.data()), bytes);
    }

    // Volume of fresh charge per crank revolution if every chamber filled to ambient: swept
    // volume per cycle (rotor faces counted individually) over revolutions per cycle
    double sweptPerRevolution(Engine *engine) {
        double perCycle = engine->getDisplacement();
        for (int i = 0; i < engine->getCylinderBankCount(); ++i) {
            const CylinderBank *bank = engine->getCylinderBank(i);
            if (bank->isRotor()) perCycle += bank->rotorFaceDisplacement() * (bank->getCylinderCount() - 1);
        }
        return perCycle / (engine->getOutputCrankshaft()->getCycleLength() / (2 * constants::pi));
    }

    json benchEngine(EngineHost &host, const std::string &script, const Options &options) {
        json result = { { "script", script } };

        const EngineHost::LoadResult load = host.loadScript(script);
        if (!load.engineLoaded || load.usedFallback) {
            result["error"] = load.message + (load.errors.empty() ? "" : "\n" + load.errors);
            return result;
        }

        auto lock = host.lock();
        Simulator *simulator = host.simulator();
        Engine *engine = host.engine();
        result["name"] = engine->getName();

        Bench bench(simulator, engine);
        const double limit = units::toRpm(engine->getIgnitionModule()->getRevLimit());
        const double idleStart = options.idleStart;
        result["rev_limit"] = limit;

        engine->getIgnitionModule()->m_enabled = true;
        simulator->m_starterMotor.m_enabled = false;
        simulator->getTransmission()->changeGear(-1);
        simulator->getTransmission()->setClutchPressure(0.0);

        const auto t0 = std::chrono::steady_clock::now();
        double simulated = 0;

        if (options.coupleRpm > 0) {
            // As BeamBridge at idle: dyno off, a PI speed controller on the crank torque
            bench.hold(options.coupleRpm, 0.0);
            bench.run(1.0);
            bench.release(0.0);
            engine->setIdleControlEnabled(options.coupleIac);
            const double target = units::rpm(options.coupleRpm);
            double inertia = 0;
            for (int i = 0; i < engine->getCrankshaftCount(); ++i) inertia += engine->getCrankshaft(i)->getMomentOfInertia();
            const double w = 2 * constants::pi * options.coupleHz;
            const double kp = 2 * 0.7 * w * inertia, ki = w * w * inertia;
            double integral = 0;
            Crankshaft *crank = engine->getOutputCrankshaft();
            bench.setStepHook([&] {
                const double forward = (simulator->m_starterMotor.m_rotationSpeed > 0) ? 1.0 : -1.0;
                double error = crank->m_body.v_theta * forward - target;
                if (options.coupleFree) {
                    const double band = units::rpm(300);
                    if (std::abs(error) < band) { integral = 0; simulator->m_crankTorque.torque = 0; return; }
                    error -= (error > 0) ? band : -band;
                }
                integral = std::clamp(integral + error * simulator->getTimestep(), -500.0 / ki, 500.0 / ki);
                const double brake = std::clamp(kp * error + ki * integral, -500.0, 500.0);
                simulator->m_crankTorque.torque = -forward * brake;
            });
            const int windows = std::max(1, static_cast<int>(options.coupleSeconds / 5));
            for (int window = 0; window < windows; ++window) {
                std::string strip;
                std::map<int, int> before;
                for (int c = 0; c < engine->getCylinderCount(); ++c) { before[c] = engine->getChamber(c)->misfires(); engine->getChamber(c)->popLitLastFrame(); }
                double sum = 0, sq = 0; int frames = 0, fires = 0;
                std::map<int, int> seen = before;
                bench.run(5.0, [&] {
                    const double rpm = std::abs(engine->getRpm());
                    sum += rpm; sq += rpm * rpm; ++frames;
                    for (int c = 0; c < engine->getCylinderCount(); ++c) {
                        if (engine->getChamber(c)->misfires() != seen[c]) { seen[c] = engine->getChamber(c)->misfires(); strip += '.'; }
                        if (engine->getChamber(c)->popLitLastFrame()) { ++fires; strip += '#'; }
                    }
                });
                int misfires = 0;
                for (int c = 0; c < engine->getCylinderCount(); ++c) misfires += engine->getChamber(c)->misfires() - before[c];
                const double mean = sum / frames;
                std::printf("  coupled %2d-%2d s: rpm %5.0f +-%4.0f  firings %4d  misfires %4d (%.0f%%)\n", window * 5, window * 5 + 5,
                    mean, std::sqrt(std::max(0.0, sq / frames - mean * mean)), fires, misfires, 100.0 * misfires / std::max(1, fires + misfires));
                std::printf("    %s\n", strip.substr(0, 96).c_str());
            }
            bench.setStepHook(nullptr);
            simulator->m_crankTorque.torque = 0;
        }

        if (options.traceBurn > 0) {
            // Each firing's burn: what fraction of the charge the flame reached, how fast, and
            // how many crank degrees it took, over a second at full throttle
            bench.hold(options.traceBurn, 1.0);
            bench.run(1.0);
            const int n = engine->getCylinderCount();
            std::vector<double> startAngle(n, -1), at50(n, -1), at90(n, -1);
            double completion = 0, speed = 0, residual = 0, efficiency = 0, sum50 = 0, sum90 = 0;
            int burns = 0, n50 = 0, n90 = 0;
            Crankshaft *shaft = engine->getOutputCrankshaft();
            const double cycle = shaft->getCycleLength() * 180 / constants::pi;
            std::vector<bool> wasLit(n, false);
            // Step by step, so the burn angles are exact
            auto step = [&] {
                for (int c = 0; c < n; ++c) {
                    CombustionChamber *chamber = engine->getChamber(c);
                    const bool lit = chamber->isLit();
                    const double angle = shaft->getCycleAngle() * 180 / constants::pi;
                    const double burned = chamber->m_flameEvent.percentageLit;
                    if (lit && !wasLit[c]) { startAngle[c] = angle; at50[c] = at90[c] = -1; }
                    if (lit && startAngle[c] >= 0) {
                        const double since = std::fmod(angle - startAngle[c] + cycle, cycle);
                        if (at50[c] < 0 && burned >= 0.5) { at50[c] = since; sum50 += since; ++n50; }
                        if (at90[c] < 0 && burned >= 0.9) { at90[c] = since; sum90 += since; ++n90; }
                    }
                    if (!lit && wasLit[c] && startAngle[c] >= 0) {
                        completion += burned;
                        speed += chamber->m_flameEvent.flameSpeed;
                        residual += chamber->m_flameEvent.residualFraction;
                        efficiency += chamber->m_flameEvent.efficiency;
                        ++burns;
                    }
                    wasLit[c] = lit;
                }
            };
            double plenum = 0, collector = 0;
            long samples = 0;
            bench.setStepHook([&] {
                step();
                plenum += engine->getIntake(0)->m_system.pressure();
                collector += engine->getExhaustSystem(0)->getSystem()->pressure();
                ++samples;
            });
            bench.run(1.0);
            bench.setStepHook(nullptr);
            std::printf("  mean plenum %.1f kPa, exhaust collector %.1f kPa\n", plenum / samples / 1000, collector / samples / 1000);
            if (burns > 0) {
                std::printf("  burns at %.0f rpm WOT (%d): spark to 50%% burned %.0f deg, to 90%% %.0f deg (%d of them got there), flame reached %.0f%%, flame %.1f m/s, efficiency %.2f, residual %.0f%%\n",
                    options.traceBurn, burns, n50 ? sum50 / n50 : -1.0, n90 ? sum90 / n90 : -1.0, n90, 100 * completion / burns,
                    speed / burns, efficiency / burns, 100 * residual / burns);
            }
        }

        if (options.dyno) {
            json curve = json::array();
            for (int i = 0; i < options.points; ++i) {
                const double rpm = idleStart + (limit * 0.98 - idleStart) * i / (options.points - 1);
                bench.hold(rpm, 1.0);
                const bool boosted = engine->getCompressorStageCount() > 0;
                bench.run(boosted ? 2.5 : 0.8); // Turbos take a while to spool
                if (boosted) simulated += 1.7;

                double torque = 0, airflow = 0, boost = 0, shaft = 0;
                int frames = 0;
                SoundStats sound;
                bench.run(0.5, [&] {
                    torque += units::convert(simulator->getFilteredDynoTorque(), units::Nm);
                    airflow += engine->getIntakeFlowRate();
                    boost += engine->getManifoldPressure() - units::pressure(1.0, units::atm);
                    for (int k = 0; k < engine->getIntakeCount(); ++k) {
                        if (const ForcedInduction *fi = engine->getIntake(k)->getForcedInduction()) {
                            for (int j = 0; j < fi->stageCount(); ++j) shaft = std::max(shaft, fi->state(j).speed);
                        }
                    }
                    sound.add(bench.lastAudio(), simulator->synthesizer().getLevelerGain());
                    ++frames;
                });
                simulated += 1.3;
                if (std::getenv("ENGINE_SIM_BENCH_FI")) {
                    for (int k = 0; k < engine->getIntakeCount(); ++k) {
                        const ForcedInduction *fi = engine->getIntake(k)->getForcedInduction();
                        if (fi == nullptr) continue;
                        std::printf("    fi @%.0f: charge %+.1f kPa plenum %+.1f kPa  wg %.2f bov %.2f  exhaust %.1f kPa",
                            rpm, fi->boost() / 1000, (engine->getIntake(k)->m_system.pressure() - 101325) / 1000,
                            fi->wastegateOpening(), fi->bovOpening(),
                            fi->exhaust() ? (fi->exhaust()->getSystem()->pressure() - 101325) / 1000 : 0.0);
                        for (int j = 0; j < fi->stageCount(); ++j) {
                            const auto &st = fi->state(j);
                            std::printf(" | %.0fk flow %.3f turb %.3f tq %.4f/%.4f%s", st.speed * 60 / (2 * constants::pi) / 1000,
                                st.massFlow, st.turbineFlow, st.compressorTorque, st.turbineTorque, st.engaged ? "" : " (off)");
                        }
                        std::printf("\n");
                    }
                }
                const double ambient = units::pressure(1.0, units::atm) / (constants::R * units::celcius(25.0)); // mol/m^3
                const double ideal = sweptPerRevolution(engine) * ambient * rpm / 60.0;
                double closed = 0;
                if (options.closedCurve) {
                    const bool idleControl = true;
                    engine->setIdleControlEnabled(false); // The closed-throttle torque of the engine itself
                    bench.hold(rpm, 0.0);
                    bench.run(0.8);
                    int closedFrames = 0;
                    bench.run(0.4, [&] { closed += units::convert(simulator->getFilteredDynoTorque(), units::Nm); ++closedFrames; });
                    closed /= closedFrames;
                    engine->setIdleControlEnabled(idleControl);
                    simulated += 1.2;
                }
                curve.push_back({ { "rpm", std::round(rpm) }, { "torque", torque / frames }, { "closed", closed }, { "ve", airflow / frames / ideal },
                    { "boost", boost / frames / 1000.0 }, { "shaft", shaft * 60 / (2 * constants::pi) / 1000.0 },
                    { "loudness", sound.rms() }, { "brightness", sound.brightness() } });
            }
            result["dyno"] = curve;

            // Engine braking: closed throttle at three quarters of the limit should pull, not push
            bench.hold(limit * 0.75, 0.0);
            bench.run(1.5);
            double overrun = 0;
            int overrunFrames = 0;
            bench.run(0.5, [&] { overrun += units::convert(simulator->getFilteredDynoTorque(), units::Nm); ++overrunFrames; });
            simulated += 2.0;
            result["overrun"] = { { "rpm", std::round(limit * 0.75) }, { "torque", overrun / overrunFrames } };

            // Idle: bring it down to 1000 rpm over a second with the throttle closed, let go, and
            // see where it settles
            const double top = limit * 0.98;
            int step = 0;
            const double rampFrames = std::max(1.0, options.idleRamp * 60);
            bench.run(std::max(options.idleRamp, 1 / 60.0), [&] { bench.hold(top + (idleStart - top) * std::min(1.0, ++step / rampFrames), 0.0); });
            bench.run(0.5);
            // Held, as BeamNG holds it: the engine's own idle control is off then, as in the game
            if (options.holdIdle > 0) {
                bench.hold(options.holdIdle, 0.0);
                engine->setIdleControlEnabled(false);
            }
            else bench.release(0.0);
            int traced = 0;
            int combustionFrame = 0;
            std::string strip;
            std::map<int, int> misfires;
            for (int c = 0; c < engine->getCylinderCount(); ++c) misfires[c] = engine->getChamber(c)->misfires();
            bench.run(3.0, [&] {
                ++combustionFrame;
                if (options.traceCombustion) {
                    for (int c = 0; c < engine->getCylinderCount(); ++c) {
                        CombustionChamber *chamber = engine->getChamber(c);
                        if (chamber->misfires() != misfires[c]) {
                            misfires[c] = chamber->misfires();
                            strip += '.';
                        }
                        if (!chamber->popLitLastFrame()) continue;
                        // How fast this firing's flame is relative to an undiluted one
                        const double speed = chamber->m_flameEvent.flameSpeed / std::max(1e-9, chamber->m_flameEvent.undilutedFlameSpeed);
                        strip += speed > 0.6 ? '#' : (speed > 0.3 ? '+' : '-');
                        const auto &event = chamber->m_flameEvent;
                        std::printf("  fire t=%.3f cyl %2d  rpm %5.0f  residual %4.1f%%  phi %.2f  flame %5.1f m/s  eff %.2f  (its last burn reached %3.0f%%)\n",
                            combustionFrame / 60.0, c, engine->getRpm(), event.residualFraction * 100, event.equivalenceRatio,
                            event.flameSpeed, event.efficiency, chamber->lastBurnCompletion() * 100);
                    }
                }
                if (options.traceIdle && traced++ % 6 == 0) {
                    std::printf("  idle t=%.1f s  rpm %.0f  plenum %.1f kPa  air in %.4f mol/s  throttle %.4f\n", traced / 60.0, engine->getRpm(),
                        engine->getIntake(0)->m_system.pressure() / 1000, engine->getIntakeFlowRate(), engine->getThrottle());
                }
            });
            if (options.traceCombustion) {
                // Burned gas in each intake runner (pushed back from the cylinder) and in the plenum
                double runner = 0;
                for (int c = 0; c < engine->getCylinderCount(); ++c) {
                    const GasSystem::Mix m = engine->getChamber(c)->intakeRunnerMix();
                    runner += std::clamp((m.p_inert - 3 * m.p_o2) / std::max(1e-9, 1 - m.p_fuel), 0.0, 1.0);
                }
                const GasSystem::Mix plenum = engine->getIntake(0)->m_system.mix();
                std::printf("  burned gas now: intake runners %.0f%% on average, plenum %.0f%% (plenum at %.0f kPa)\n",
                    100 * runner / engine->getCylinderCount(),
                    100 * std::clamp((plenum.p_inert - 3 * plenum.p_o2) / std::max(1e-9, 1 - plenum.p_fuel), 0.0, 1.0),
                    engine->getIntake(0)->m_system.pressure() / 1000);
                int total = 0;
                for (int c = 0; c < engine->getCylinderCount(); ++c) total += engine->getChamber(c)->misfires();
                std::printf("  firing strip, flame speed vs undiluted (# >60%, + 30-60%, - <30%, . misfire):\n");
                for (size_t i = 0; i < strip.size(); i += 96) std::printf("    %s\n", strip.substr(i, 96).c_str());
            }
            double sum = 0, sumSquares = 0;
            int n = 0;
            bench.run(2.0, [&] {
                const double rpm = std::abs(engine->getRpm());
                sum += rpm;
                sumSquares += rpm * rpm;
                ++n;
            });
            simulated += 5.5;
            const double mean = sum / n;
            result["idle"] = { { "rpm", mean }, { "wobble", std::sqrt(std::max(0.0, sumSquares / n - mean * mean)) } };
        }

        if (!options.wavDir.empty() && options.startStop) {
            // From rest: the starter cranks until the engine fires, then idles, then is switched off
            bench.release(0.0);
            simulator->m_dyno.m_rotationSpeed = 0;
            engine->getOutputCrankshaft()->m_body.v_theta = 0;
            engine->getIgnitionModule()->m_enabled = true;
            bench.record(true);
            double t = 0;
            int traceFrame = 0;
            bench.run(6.0, [&] {
                t += 1 / 60.0;
                simulator->m_starterMotor.m_enabled = t < 1.0 || (t < 5.0 && std::abs(engine->getRpm()) < 400);
                if (options.traceIdle && traceFrame++ % 15 == 0) {
                    double raw = 0;
                    for (int16_t x : bench.lastAudio()) raw += double(x) * x;
                    raw = std::sqrt(raw / std::max<size_t>(1, bench.lastAudio().size()));
                    std::printf("  start t=%.2f rpm %5.0f starter %d leveler gain %.4f output rms %.0f\n", t, engine->getRpm(),
                        simulator->m_starterMotor.m_enabled ? 1 : 0, simulator->synthesizer().getLevelerGain(), raw);
                }
            });
            simulator->m_starterMotor.m_enabled = false;
            bench.run(3.0);
            engine->getIgnitionModule()->m_enabled = false;
            bench.run(4.0);
            bench.record(false);
            simulated += 13.0;
        }
        else if (options.liftRpm > 0) {
            bench.hold(options.liftRpm, 1.0);
            bench.run(2.0);
            bench.record(true);
            double t = 0;
            auto trace = [&] {
                t += 1 / 60.0;
                for (int k = 0; k < engine->getIntakeCount(); ++k) {
                    const ForcedInduction *fi = engine->getIntake(k)->getForcedInduction();
                    if (fi == nullptr) continue;
                    std::printf("  lift t=%.2f thr %.0f  charge %+6.1f plenum %+6.1f kPa  bov %.2f wg %.2f",
                        t, engine->getSpeedControl() * 100, fi->boost() / 1000,
                        (engine->getIntake(k)->m_system.pressure() - 101325) / 1000, fi->bovOpening(), fi->wastegateOpening());
                    for (int j = 0; j < fi->stageCount(); ++j) {
                        std::printf(" | %3.0fk %+.3f kg/s", fi->state(j).speed * 60 / (2 * constants::pi) / 1000, fi->state(j).massFlow);
                    }
                    std::printf("\n");
                }
            };
            bench.run(1.0, trace);
            bench.hold(options.liftRpm, 0.0);
            bench.run(2.0, trace);
            bench.hold(options.liftRpm, 1.0);
            bench.run(1.5, trace);
            bench.record(false);
            simulated += 6.5;
        }
        else if (!options.wavDir.empty() && options.steadyRpm > 0) {
            bench.hold(options.steadyRpm, options.steadyThrottle);
            bench.run(2.0);
            bench.record(true);
            bench.run(4.0);
            bench.record(false);
            simulated += 6.0;
        }
        else if (!options.wavDir.empty()) {
            // Idle, a full-throttle run-up held by the dyno, then a lift back down
            bench.hold(options.holdIdle > 0 ? options.holdIdle : idleStart, 0.0);
            engine->setIdleControlEnabled(options.holdIdle <= 0);
            bench.run(0.5);
            if (options.holdIdle <= 0) bench.release(0.0);
            bench.run(1.0);
            bench.record(true);
            bench.run(2.0);
            const double from = std::max(std::abs(engine->getRpm()), 600.0);
            const double runUp = 5.0;
            int frame = 0;
            bench.run(runUp, [&] {
                bench.hold(from + (limit * 0.97 - from) * (++frame / (runUp * 60)), 1.0);
            });
            frame = 0;
            bench.run(3.0, [&] {
                bench.hold(limit * 0.97 + (from - limit * 0.97) * std::min(1.0, ++frame / 150.0), 0.0);
            });
            if (options.holdIdle > 0) bench.hold(options.holdIdle, 0.0);
            else bench.release(0.0);
            bench.run(1.0);
            bench.record(false);
            simulated += 13.5;
        }

        if (!options.wavDir.empty()) {
            std::filesystem::create_directories(options.wavDir);
            const std::string name = std::filesystem::path(script).stem().string();
            const std::string path = (std::filesystem::path(options.wavDir) / (name + ".wav")).string();
            writeWav(path, bench.audio(), 44100);
            result["wav"] = path;
        }

        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        result["speed"] = wall > 0 ? simulated / wall : 0; // times real time
        return result;
    }

    // Prints what moved; true if everything is within tolerance
    bool compare(const json &now, const json &before, double tolerancePercent) {
        bool ok = true;
        for (const json &engine : now) {
            const std::string script = engine.value("script", "");
            const auto old = std::find_if(before.begin(), before.end(), [&](const json &b) { return b.value("script", "") == script; });
            if (old == before.end()) {
                std::printf("  %s: new\n", script.c_str());
                continue;
            }
            if (engine.contains("error") != old->contains("error")) {
                std::printf("  %s: %s\n", script.c_str(), engine.contains("error") ? "now fails to load" : "now loads");
                ok = false;
                continue;
            }
            if (!engine.contains("dyno") || !old->contains("dyno")) continue;

            const json &a = engine["dyno"], &b = (*old)["dyno"];
            double peak = 1;
            for (const json &p : b) peak = std::max(peak, std::abs(p.value("torque", 0.0)));

            double worst = 0, worstRpm = 0;
            for (size_t i = 0; i < std::min(a.size(), b.size()); ++i) {
                const double change = std::abs(a[i].value("torque", 0.0) - b[i].value("torque", 0.0)) / peak * 100;
                if (change > worst) { worst = change; worstRpm = a[i].value("rpm", 0.0); }
            }
            const double idleNow = engine["idle"].value("rpm", 0.0), idleBefore = (*old)["idle"].value("rpm", 0.0);
            const bool idleMoved = std::abs(idleNow - idleBefore) > std::max(50.0, idleBefore * 0.05);
            const bool moved = worst > tolerancePercent || a.size() != b.size() || idleMoved;
            if (moved) ok = false;
            std::printf("  %-50s torque %s (worst %.1f%% at %.0f rpm), idle %.0f -> %.0f rpm\n",
                script.c_str(), moved ? "CHANGED" : "same   ", worst, worstRpm, idleBefore, idleNow);
        }
        return ok;
    }
}

int main(int argc, char **argv) {
    Options options;
    if (!parse(argc, argv, &options)) return 2;

    EngineHost host(HostPaths::resolve());
    host.setOutputHeadroom(6); // As the default settings have it
    if (options.levelerRelease > 0) host.setLevelerRelease(options.levelerRelease);

    json results = json::array();
    for (const std::string &script : options.scripts) {
        const json r = benchEngine(host, script, options);
        results.push_back(r);

        if (r.contains("error")) {
            std::printf("%s: failed to load: %s\n", script.c_str(), r["error"].get<std::string>().c_str());
            continue;
        }
        std::printf("%s (%s), %.1fx real time\n", r.value("name", "").c_str(), script.c_str(), r.value("speed", 0.0));
        if (r.contains("dyno")) {
            bool boosted = false;
            for (const json &p : r["dyno"]) boosted = boosted || p.value("shaft", 0.0) > 0;
            std::printf("  %8s %10s %6s %10s %10s%s%s\n", "rpm", "torque Nm", "VE", "loudness", "bright",
                boosted ? "  boost kPa  shaft krpm" : "", options.closedCurve ? "  closed Nm" : "");
            for (const json &p : r["dyno"]) {
                std::printf("  %8.0f %10.1f %5.0f%% %10.0f %10.3f", p.value("rpm", 0.0), p.value("torque", 0.0),
                    100 * p.value("ve", 0.0), p.value("loudness", 0.0), p.value("brightness", 0.0));
                if (boosted) std::printf(" %10.1f %11.0f", p.value("boost", 0.0), p.value("shaft", 0.0));
                if (options.closedCurve) std::printf(" %10.1f", p.value("closed", 0.0));
                std::printf("\n");
            }
            std::printf("  idle %.0f rpm (wobble %.0f)\n", r["idle"].value("rpm", 0.0), r["idle"].value("wobble", 0.0));
            if (r.contains("overrun")) {
                std::printf("  closed throttle at %.0f rpm: %.1f Nm\n", r["overrun"].value("rpm", 0.0), r["overrun"].value("torque", 0.0));
            }
        }
        if (r.contains("wav")) std::printf("  wrote %s\n", r["wav"].get<std::string>().c_str());
        std::fflush(stdout);
    }

    if (!options.jsonOut.empty()) {
        std::ofstream(options.jsonOut) << results.dump(1) << "\n";
    }

    if (!options.compareWith.empty()) {
        std::ifstream file(options.compareWith);
        const json before = json::parse(file, nullptr, false);
        if (!before.is_array()) {
            std::fprintf(stderr, "engine-sim-bench: can't read %s\n", options.compareWith.c_str());
            return 2;
        }
        std::printf("Compared with %s:\n", options.compareWith.c_str());
        return compare(results, before, options.tolerance) ? 0 : 1;
    }
    return 0;
}
