#include "direct_throttle_linkage.h"

#include "engine.h"
#include "ignition_module.h"

#include <algorithm>
#include <cmath>

DirectThrottleLinkage::DirectThrottleLinkage() {
    m_gamma = 1.0;
    m_throttlePosition = 1.0;
}

DirectThrottleLinkage::~DirectThrottleLinkage() {
    /* void */
}

void DirectThrottleLinkage::initialize(const Parameters &params) {
    m_gamma = params.gamma;
    m_idleSpeed = params.idleSpeed;
    m_idleMaxOpening = params.idleMaxOpening;
    m_idleProportional = params.idleProportional;
    m_idleIntegral = params.idleIntegral;
    m_idleFuelCutRatio = params.idleFuelCutRatio;
}

void DirectThrottleLinkage::setSpeedControl(double s) {
    Throttle::setSpeedControl(s);
    m_throttlePosition = 1 - std::pow(s, m_gamma);
}

void DirectThrottleLinkage::update(double dt, Engine *engine) {
    Throttle::update(dt, engine);

    // Overrun fuel cut, as fuel injection does, with a little hysteresis: off throttle above the
    // idle it fires nothing, so the overrun is a smooth pumping tone rather than diluted charges
    // misfiring at random, and big-overlap engines whose idle air would drive them on don't run
    // away. It works whether or not the idle air control is in charge.
    const double speed = std::abs(engine->getSpeed());
    bool &cut = engine->getIgnitionModule()->m_fuelCut;
    if (m_idleSpeed > 0 && m_idleFuelCutRatio > 0 && m_speedControl < 0.01) {
        if (speed > m_idleFuelCutRatio * m_idleSpeed) cut = true;
        else if (speed < (0.5 + 0.5 * m_idleFuelCutRatio) * m_idleSpeed) cut = false;
    }
    else {
        cut = false;
    }

    if (m_idleSpeed <= 0 || !m_idleControlEnabled) {
        m_idleOpening = 0;
        engine->setThrottle(m_throttlePosition);
        return;
    }

    // PI control on the relative speed error, both ways: like an idle air valve, it supplies
    // the idle air (so give the throttle plate little leakage of its own) and trims it up or
    // down. The pedal always wins when it asks for more.
    const double error = (m_idleSpeed - std::abs(engine->getSpeed())) / m_idleSpeed;
    m_idleOpening = std::clamp(m_idleOpening + m_idleIntegral * error * dt, 0.0, m_idleMaxOpening);
    double opening = m_idleOpening + m_idleProportional * error;
    // Stall saver: a gentle controller keeps a lumpy idle lumpy instead of chasing every lurch
    // into a surge, but it's too slow to catch an engine that has stopped firing. Well below
    // the idle it opens right up.
    if (error > 0.1) opening = std::max(opening, 4 * (error - 0.1) * m_idleMaxOpening / 0.25);
    opening = std::clamp(opening, 0.0, m_idleMaxOpening);

    const double pedal = std::max(m_speedControl, opening);
    engine->setThrottle(1 - std::pow(pedal, m_gamma));
}
