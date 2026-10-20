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
    m_active = false;
    m_tuning = false;
    m_profileValid = false;
    m_profileStep = 0;
    m_profileTimer = 0;
    m_profileId += 1;
    m_dynoHistory.clear();
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
        m_logMessage = "BEAMNG BRIDGE DISCONNECTED";
    }

    m_active = false;
    m_tuning = false;
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

    if (m_retune) {
        m_retune = false;
        m_profileValid = false;
        m_profileStep = 0;
        m_profileTimer = 0;
        m_profileId += 1;
    }

    const bool sweeping = m_settings.tuneBeamEngine && !m_profileValid;
    m_tuning = sweeping;

    double targetRpm = smoothRpm;
    double throttle = m_input.throttle;
    if (sweeping) runProfileSweep(engine, simulator, dt, &targetRpm, &throttle);

    // Hold the crank at BeamNG's speed, ramping across the tick in onStep()
    m_rampFrom = m_rampTo;
    m_rampTo = units::rpm(std::max(targetRpm, 0.0));
    simulator->m_dyno.m_enabled = true;
    simulator->m_dyno.m_hold = true;
    simulator->m_dyno.m_forwardSign = (simulator->m_starterMotor.m_rotationSpeed > 0) ? 1.0 : -1.0;
    simulator->m_dyno.m_rotationSpeed = m_rampFrom;
    simulator->m_starterMotor.m_enabled = false;
    engine->getIgnitionModule()->m_enabled = sweeping || m_input.ignition > 0.5f;

    engine->setSpeedControl(clamp(throttle));

    // engine-sim's own drivetrain isn't used: the car is BeamNG's
    if (simulator->getTransmission()->getGear() != -1) {
        simulator->getTransmission()->changeGear(-1);
    }
    simulator->getTransmission()->setClutchPressure(0.0);

    // The dyno torque includes spinning up engine-sim's own crank. Add I*alpha back, both
    // averaged over the same window, or the leftover kicks BeamNG's rpm and the loop buzzes.
    const double inertia = totalCrankInertia(engine);
    const double cycleSeconds = 4 * constants::pi / std::max(m_rampTo, 10.0);
    const double window = std::max(2 * cycleSeconds, 0.05);
    const double alpha = units::rpm(m_transport->rpmRate(window));

    const double now = nowSeconds();
    m_dynoHistory.emplace_back(now, simulator->getFilteredDynoTorque());
    while (m_dynoHistory.front().first < now - window) m_dynoHistory.pop_front();

    double dynoMean = 0;
    for (const auto &sample : m_dynoHistory) dynoMean += sample.second;
    dynoMean /= m_dynoHistory.size();
    const double engineTorque = dynoMean + inertia * alpha;

    // NaN keeps BeamNG's torque: at idle its controller hunts against ours, and off throttle ours trails the rpm
    // The game only uses the newest value; ~250 Hz is plenty (BeamNG runs updateGFX per frame)
    m_torqueSendTimer += dt;
    if (m_torqueSendTimer >= 1 / 250.0) {
        m_torqueSendTimer = 0;

        const bool beamIdles = sweeping || m_input.rpm < m_settings.beamIdleRpm || m_input.throttle < 0.02f;
        m_transport->sendTorque(
            beamIdles ? std::nanf("") : static_cast<float>(units::convert(engineTorque, units::Nm)),
            static_cast<float>(units::toRpm(m_rampTo)));
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
    }
}

void BeamBridge::onStep(Simulator *simulator, int step, int iterationCount) {
    if (!m_active) return;

    const double f = std::min(1.0, static_cast<double>(step) / std::max(iterationCount, 1));
    simulator->m_dyno.m_rotationSpeed = m_rampFrom + (m_rampTo - m_rampFrom) * f;
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

    char message[128];
    std::snprintf(message, sizeof(message), "BEAMNG PROFILE: IDLE %.0f RPM, LIMIT %.0f RPM", idle, limit);
    m_logMessage = message;
}
