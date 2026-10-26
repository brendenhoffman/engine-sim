#include "beam_bridge.h"

#include "constants.h"
#include "crankshaft.h"
#include "engine.h"
#include "ignition_module.h"
#include "simulator.h"
#include "transmission.h"
#include "units.h"
#include "utilities.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>

namespace {
    constexpr int ProfilePoints = 24;

    double nowSeconds() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    double profilePointRpm(int i, double limitRpm) {
        const double top = limitRpm * 0.97;
        return 400.0 + i * (top - 400.0) / (ProfilePoints - 1);
    }

    double totalCrankInertia(Engine *engine) {
        double inertia = 0;
        for (int i = 0; i < engine->getCrankshaftCount(); ++i) {
            inertia += engine->getCrankshaft(i)->getMomentOfInertia();
        }
        return inertia;
    }
}

BeamBridge::BeamBridge(BridgeTransport *transport) : m_transport(transport) {
    /* void */
}

void BeamBridge::retune() {
    m_retune = true;
}

void BeamBridge::reset() {
    m_calibration.clear();
    m_followRpm = -1;
    m_active = false;
    m_tuning = false;
    m_profileValid = false;
    m_profileStep = 0;
    m_profileTimer = 0;
    m_profileId += 1;
    m_dynoHistory.clear();
    m_spinHistory.clear();
}

std::string BeamBridge::takeLogMessage() {
    std::string message;
    std::swap(message, m_logMessage);
    return message;
}

void BeamBridge::deactivate(Simulator *simulator, Engine *engine) {
    if (m_active) {
        // Hand the engine back idling, not at whatever throttle the game last asked for
        simulator->m_dyno.m_enabled = false;
        simulator->m_dyno.m_hold = false;
        simulator->m_dyno.m_forwardSign = 0;
        engine->setSpeedControl(0.0);
        engine->setIdleControlEnabled(true);
        simulator->m_crankTorque.torque = 0;
        m_logMessage = "BEAMNG BRIDGE DISCONNECTED";
    }

    m_active = false;
    m_tuning = false;
    m_followRpm = -1;
    m_idleCoupling = false;
}

void BeamBridge::beforeFrame(Simulator *simulator, Engine *engine, double dt) {
    double age = 1e9;
    float smoothRpm = 0;
    if (!m_transport->latest(&m_input, &age, &smoothRpm)) age = 1e9;

    if (!m_settings.connect || age >= 0.5) {
        deactivate(simulator, engine);
        return;
    }

    if (!m_active) {
        m_logMessage = "BEAMNG BRIDGE CONNECTED";
        m_rampTo = std::abs(engine->getSpeed());
    }
    m_active = true;

    // Which car is BeamNG driving? A new one needs measuring again
    m_vehicleTimer += dt;
    if (m_vehicleTimer > 0.5) {
        m_vehicleTimer = 0;
        refreshVehicle();
    }

    if (m_retune) {
        m_retune = false;
        m_profileValid = false;
        m_profileStep = 0;
        m_profileTimer = 0;
        m_profileId += 1;
    }

    // The sweep measures engine-sim's full-throttle curve: for retuning the car, or for matching
    // the car's power in drive mode
    const bool calibrating = m_settings.driveCar && m_settings.matchCarPower && !m_settings.tuneBeamEngine && !m_beamTorque.empty();
    const bool sweeping = (m_settings.tuneBeamEngine || calibrating) && !m_profileValid;
    m_tuning = sweeping;

    // Follow BeamNG's rpm smoothly: a critically damped spring (~20 Hz) on the extrapolated packets
    if (m_followRpm < 0) {
        m_followRpm = smoothRpm;
        m_followRate = 0;
    }
    else {
        const double w = 2 * constants::pi * 20.0;
        const double h = std::min(dt, 0.01);
        m_followRate += (w * w * (smoothRpm - m_followRpm) - 2 * w * m_followRate) * h;
        m_followRpm += m_followRate * h;
    }
    double targetRpm = sweeping ? smoothRpm : m_followRpm;
    double throttle = m_input.throttle;
    if (sweeping) runProfileSweep(engine, simulator, dt, &targetRpm, &throttle);

    // Hold the crank at BeamNG's speed, ramping across the tick in onStep()
    m_rampFrom = m_rampTo;
    m_rampTo = units::rpm(std::max(targetRpm, 0.0));
    simulator->m_dyno.m_enabled = true;
    simulator->m_dyno.m_hold = true;

    // Near idle with the throttle shut, follow BeamNG's speed loosely instead of holding it:
    // a slow speed controller keeps the average where the game's idle is, while each firing
    // and misfire still shakes the crank, as a free-running lumpy idle does
    {
        const double idleRpm = engine->getIdleRpm() > 0 ? units::toRpm(engine->getIdleRpm())
            : (m_beamIdleRpm > 0 ? m_beamIdleRpm : 900.0);
        const bool free = m_settings.idleMode == 2 || (m_settings.idleMode == 0 && engine->getIdleFreeRunning());
        const bool atIdle = !sweeping && (free || m_settings.idleFollowHz > 0) && throttle < 0.02 && targetRpm < 1.3 * idleRpm;
        if (atIdle && !m_idleCoupling) m_couplingIntegral = 0;
        m_idleCoupling = atIdle;
        m_idleFree = atIdle && free;
        m_couplingTarget = units::rpm(std::max(targetRpm, 0.0));
        if (atIdle) {
            simulator->m_dyno.m_enabled = false;
            simulator->m_dyno.m_hold = false;
        }
        else {
            simulator->m_crankTorque.torque = 0;
        }
    }
    simulator->m_dyno.m_forwardSign = (simulator->m_starterMotor.m_rotationSpeed > 0) ? 1.0 : -1.0;
    simulator->m_dyno.m_rotationSpeed = m_rampFrom;
    simulator->m_starterMotor.m_enabled = false;
    engine->getIgnitionModule()->m_enabled = sweeping || m_input.ignition > 0.5f;

    engine->setSpeedControl(clamp(throttle));
    // Away from idle BeamNG sets the speed. At idle the engine's own idle air control runs (as
    // on its own), under the coupling or freely.
    engine->setIdleControlEnabled(m_idleCoupling);

    // engine-sim's own drivetrain isn't used: the car is BeamNG's
    if (simulator->getTransmission()->getGear() != -1) {
        simulator->getTransmission()->changeGear(-1);
    }
    simulator->getTransmission()->setClutchPressure(0.0);

    // The dyno torque includes spinning up engine-sim's own crank to BeamNG's speed. We know
    // exactly how hard we spun it, so take that out over the same stretch of time the dyno
    // reading covers (its last engine cycle). What's left is the engine's own torque, which is
    // what the car's engine should make; the car's flywheel is BeamNG's business. (Estimating
    // the spin-up from BeamNG's rpm instead lagged differently, and the leftover made the rpm
    // hunt, worst when engine-sim's crank is heavier than the car's flywheel.)
    const double inertia = totalCrankInertia(engine);
    const double cycleSeconds = engine->getOutputCrankshaft()->getCycleLength() / std::max(m_rampFrom, 10.0);
    const double window = std::max(2 * cycleSeconds, 0.05);
    const double now = nowSeconds();

    double spun = 0, spunTime = 0, previous = now;
    for (auto it = m_spinHistory.rbegin(); it != m_spinHistory.rend() && previous > now - cycleSeconds; ++it) {
        const double from = std::max(it->first, now - cycleSeconds);
        spun += it->second * (previous - from);
        spunTime += previous - from;
        previous = it->first;
    }
    const double spinTorque = spunTime > 0 ? inertia * spun / spunTime : 0;

    m_dynoHistory.emplace_back(now, simulator->getFilteredDynoTorque() + spinTorque);
    while (m_dynoHistory.front().first < now - window) m_dynoHistory.pop_front();

    // This frame's spin-up, for the readings to come
    const double frameSeconds = std::clamp(now - m_lastFrameTime, 1e-4, 0.05);
    m_lastFrameTime = now;
    m_spinHistory.emplace_back(now, (m_rampTo - m_rampFrom) / frameSeconds);
    while (m_spinHistory.size() > 1 && m_spinHistory[1].first < now - 0.5) m_spinHistory.pop_front();

    double engineTorque = 0;
    for (const auto &sample : m_dynoHistory) engineTorque += sample.second;
    engineTorque /= m_dynoHistory.size();

    // Matched power: scale by how the car's own curve compares with engine-sim's at this rpm
    if (calibrating && m_profileValid && !m_calibration.empty()) {
        engineTorque *= calibrationAt(units::toRpm(m_rampTo), m_profileLimitRpm);
    }

    // NaN keeps BeamNG's torque: at idle its controller hunts against ours, and off throttle ours trails the rpm
    // The game only uses the newest value; ~250 Hz is plenty (BeamNG runs updateGFX per frame)
    m_torqueSendTimer += dt;
    if (m_torqueSendTimer >= 1 / 250.0) {
        m_torqueSendTimer = 0;

        // Sound only unless asked; even then BeamNG keeps idle and closed-throttle torque
        const bool beamIdles = !m_settings.driveCar || sweeping
            || m_input.rpm < m_settings.beamIdleRpm || m_input.throttle < 0.02f;
        // A Jake brake slows the truck: its braking goes to the game even when engine-sim is
        // only making the sound, used only where it brakes harder than the game's own engine
        const bool jake = engine->isJakeBrakeActive() && !sweeping;
        m_transport->sendTorque(
            (beamIdles && !jake) ? std::nanf("") : static_cast<float>(units::convert(engineTorque, units::Nm)),
            static_cast<float>(units::toRpm(m_rampTo)),
            static_cast<float>((engine->getManifoldPressure() - units::pressure(1.0, units::atm)) / 1000.0),
            (beamIdles && jake) ? 1.0f : 0.0f);
    }

    m_profileSendTimer += dt;
    if (m_profileSendTimer > 0.5) {
        m_profileSendTimer = 0;
        if (m_settings.tuneBeamEngine && m_profileValid) {
            m_transport->sendProfile(m_profilePacket.data(), static_cast<int>(m_profilePacket.size()));
        }
        else {
            const float off[2] = { 0, m_profileId };
            m_transport->sendProfile(off, 2);
        }

        m_transport->sendSoundConfig(m_settings.neutralEq, m_settings.intakeLevel, m_settings.exhaustLevel, sweeping, m_settings.gameBurble);
        m_transport->sendPluginConfig(m_settings.keepGameLowCut);
    }
}

void BeamBridge::onStep(Simulator *simulator, int step, int iterationCount) {
    if (!m_active) return;

    const double f = std::min(1.0, static_cast<double>(step) / std::max(iterationCount, 1));
    simulator->m_dyno.m_rotationSpeed = m_rampFrom + (m_rampTo - m_rampFrom) * f;

    if (m_idleCoupling) {
        // PI on the crank speed, tuned from the crank's inertia for a ~idle_follow_hz response
        Engine *engine = simulator->getEngine();
        Crankshaft *crank = engine->getOutputCrankshaft();
        const double inertia = totalCrankInertia(engine);
        const double w = 2 * constants::pi * m_settings.idleFollowHz;
        const double kp = 2 * 0.7 * w * inertia, ki = w * w * inertia;
        // Speed along the engine's own direction of rotation (the starter's), so a crank that's
        // stopped or turning backwards is pushed forwards: taking the direction from the crank
        // itself spun a stopped engine up in reverse, where it never fires
        const double forward = (simulator->m_starterMotor.m_rotationSpeed > 0) ? 1.0 : -1.0;
        const double speed = crank->m_body.v_theta * forward;
        double error = speed - m_couplingTarget; // + : too fast
        if (m_idleFree) {
            // Free-running: hands off within 300 rpm of the game's idle; outside it, steer back
            const double band = units::rpm(300);
            if (std::abs(error) < band) {
                m_couplingIntegral = 0;
                simulator->m_crankTorque.torque = 0;
                return;
            }
            error -= (error > 0) ? band : -band;
        }
        m_couplingIntegral = std::clamp(m_couplingIntegral + error * simulator->getTimestep(), -500.0 / std::max(ki, 1e-6), 500.0 / std::max(ki, 1e-6));
        const double brake = std::clamp(kp * error + ki * m_couplingIntegral, -500.0, 500.0);
        simulator->m_crankTorque.torque = -forward * brake;
    }
}

void BeamBridge::onAudio(const int16_t *samples, int count) {
    // The sweep doesn't sound like driving, so the car stays quiet while profiling
    if (m_active && !m_tuning) {
        m_transport->sendAudio(samples, count, m_settings.volume);
    }
}

void BeamBridge::runProfileSweep(Engine *engine, Simulator *simulator, double dt, double *rpm, double *throttle) {
    const int n = ProfilePoints;
    const double limit = units::toRpm(engine->getIgnitionModule()->getRevLimit());
    if (m_profileStep == 0 && m_profileTimer == 0) {
        m_profileWot.assign(n, 0);
        m_profileClosed.assign(n, 0);
        m_logMessage = "PROFILING ENGINE FOR BEAMNG";
    }

    // Up the rpm range at full throttle, then back down closed
    const bool wot = m_profileStep < n;
    const int point = wot ? m_profileStep : 2 * n - 1 - m_profileStep;
    *rpm = profilePointRpm(point, limit);
    *throttle = wot ? 1.0 : 0.0;

    m_profileTimer += dt;
    const double settle = 0.12 + 2 * 120.0 / *rpm + ((m_profileStep == 0 || m_profileStep == n) ? 0.5 : 0.0);
    if (m_profileTimer < settle) return;

    (wot ? m_profileWot : m_profileClosed)[point] =
        static_cast<float>(units::convert(simulator->getFilteredDynoTorque(), units::Nm));
    m_profileTimer = 0;
    if (++m_profileStep == 2 * n) finishProfile(engine);
}

// Packet: 1, id, idle rpm, rev limit rpm, inertia kg m^2, friction Nm, dynamic friction Nm/(rad/s),
// point count, rpm[count], gross full-throttle torque Nm[count] (BeamNG subtracts the friction terms)
void BeamBridge::finishProfile(Engine *engine) {
    const int n = ProfilePoints;
    const double limit = units::toRpm(engine->getIgnitionModule()->getRevLimit());
    const double rpmToAv = constants::pi / 30;

    // Idle: where closed-throttle torque crosses zero
    double idle = 0;
    for (int i = 0; i + 1 < n && idle == 0; ++i) {
        const double a = m_profileClosed[i], b = m_profileClosed[i + 1];
        if (a > 0 && b <= 0) {
            idle = profilePointRpm(i, limit) + (profilePointRpm(i + 1, limit) - profilePointRpm(i, limit)) * a / (a - b);
        }
    }
    if (idle == 0) idle = 800;
    idle = std::max(idle, 500.0);

    // The engine's own idle speed, when its script gives one, beats the estimate: lumpy
    // big-overlap idles cross zero torque somewhere unhelpful
    if (engine->getIdleRpm() > 0) idle = units::toRpm(engine->getIdleRpm());

    // Friction: a line fitted to the closed-throttle drag above idle
    double sn = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; ++i) {
        if (profilePointRpm(i, limit) < idle * 1.3) continue;

        const double x = profilePointRpm(i, limit) * rpmToAv, y = -m_profileClosed[i];
        sn += 1; sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    double dynamicFriction = (sn > 1 && sn * sxx - sx * sx > 0) ? (sn * sxy - sx * sy) / (sn * sxx - sx * sx) : 0;
    if (dynamicFriction < 0) dynamicFriction = 0;
    const double friction = std::max(sn > 0 ? (sy - dynamicFriction * sx) / sn : 0.0, 0.0);

    const double inertia = totalCrankInertia(engine);

    m_profilePacket = { 1, m_profileId, (float)idle, (float)limit, (float)inertia, (float)friction, (float)dynamicFriction, (float)n };
    for (int i = 0; i < n; ++i) m_profilePacket.push_back((float)profilePointRpm(i, limit));
    for (int i = 0; i < n; ++i) {
        const double gross = m_profileWot[i] + friction + dynamicFriction * profilePointRpm(i, limit) * rpmToAv;
        m_profilePacket.push_back((float)std::max(gross, 0.0));
    }
    m_profileValid = true;

    // For diagnosing what a profile measured
    if (std::getenv("ENGINE_SIM_PROFILE_LOG") != nullptr) {
        std::fprintf(stderr, "profile: idle %.0f, friction %.1f Nm + %.4f Nm/(rad/s)\n", idle, friction, dynamicFriction);
        for (int i = 0; i < n; ++i) {
            std::fprintf(stderr, "  %6.0f rpm  wot %7.1f  closed %7.1f\n", profilePointRpm(i, limit), m_profileWot[i], m_profileClosed[i]);
        }
    }

    // Car torque / engine-sim torque at each point, for matched power in drive mode
    m_profileLimitRpm = limit;
    m_calibration.assign(n, 1.0);
    double sum = 0;
    int counted = 0;
    for (int i = 0; i < n; ++i) {
        const double car = beamTorqueAt(profilePointRpm(i, limit));
        const double sim = m_profileWot[i];
        if (car > 0 && sim > 5) {
            m_calibration[i] = std::clamp(car / sim, 0.2, 3.0);
            sum += m_calibration[i];
            ++counted;
        }
    }

    char message[128];
    if (m_settings.tuneBeamEngine || counted == 0) {
        std::snprintf(message, sizeof(message), "BEAMNG PROFILE: IDLE %.0f RPM, LIMIT %.0f RPM", idle, limit);
    }
    else {
        std::snprintf(message, sizeof(message), "MATCHED TO THE CAR'S POWER (x%.2f AVERAGE)", sum / counted);
    }
    m_logMessage = message;
}

void BeamBridge::refreshVehicle() {
    nlohmann::json vehicle;
    double age = 0;
    if (!m_transport->vehicleInfo(&vehicle, &age)) return;

    const nlohmann::json car = vehicle.value("vehicle", nlohmann::json::object());
    const nlohmann::json engine = vehicle.value("engine", nlohmann::json());
    const std::string key = car.value("directory", std::string()) + "|" + car.value("configuration", std::string())
        + "|" + (engine.is_object() ? engine.value("part", std::string()) : std::string());
    if (key == m_vehicleKey) return;
    m_vehicleKey = key;

    m_beamTorque.clear();
    m_beamInertia = 0;
    m_beamIdleRpm = 0;
    if (engine.is_object()) {
        const nlohmann::json inertia = engine.value("inertia", nlohmann::json());
        if (inertia.is_number()) m_beamInertia = inertia.get<double>();
        const nlohmann::json idle = engine.value("idle_rpm", nlohmann::json());
        if (idle.is_number()) m_beamIdleRpm = idle.get<double>();
        for (const nlohmann::json &row : engine.value("torque", nlohmann::json::array())) {
            if (row.is_array() && row.size() >= 2 && row[0].is_number() && row[1].is_number()) {
                m_beamTorque.emplace_back(row[0].get<double>(), row[1].get<double>());
            }
        }
        std::sort(m_beamTorque.begin(), m_beamTorque.end());
    }

    // A different car: measure again (its curve, and its stock values for retuning, differ)
    m_profileValid = false;
    m_profileStep = 0;
    m_profileTimer = 0;
    m_profileId += 1;
    m_calibration.clear();
}

double BeamBridge::beamTorqueAt(double rpm) const {
    if (m_beamTorque.empty()) return 0;
    if (rpm <= m_beamTorque.front().first) return m_beamTorque.front().second;
    for (size_t i = 1; i < m_beamTorque.size(); ++i) {
        const auto &a = m_beamTorque[i - 1], &b = m_beamTorque[i];
        if (rpm <= b.first) {
            const double t = b.first > a.first ? (rpm - a.first) / (b.first - a.first) : 0;
            return a.second + (b.second - a.second) * t;
        }
    }
    return m_beamTorque.back().second;
}

double BeamBridge::calibrationAt(double rpm, double limitRpm) const {
    const int n = static_cast<int>(m_calibration.size());
    if (n == 0) return 1.0;

    const double first = profilePointRpm(0, limitRpm), last = profilePointRpm(n - 1, limitRpm);
    const double position = std::clamp((rpm - first) / (last - first) * (n - 1), 0.0, n - 1.0);
    const int i = std::min(static_cast<int>(position), n - 2);
    const double t = position - i;
    return m_calibration[i] + (m_calibration[i + 1] - m_calibration[i]) * t;
}
