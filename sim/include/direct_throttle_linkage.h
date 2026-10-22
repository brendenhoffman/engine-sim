#ifndef ATG_ENGINE_SIM_DIRECT_THROTTLE_LINKAGE_H
#define ATG_ENGINE_SIM_DIRECT_THROTTLE_LINKAGE_H

#include "throttle.h"

class DirectThrottleLinkage : public Throttle {
public:
    struct Parameters {
        double gamma;

        // Idle air control: below this speed with the pedal up, it cracks the throttle open
        // as much as it takes to hold it (0: off)
        double idleSpeed = 0;
        double idleMaxOpening = 0.3; // As a pedal position
        // Pedal opening per unit of relative speed error, and how fast that builds per second
        double idleProportional = 0.6;
        double idleIntegral = 0.5;
        // Overrun fuel cut, as fuel injection does: with the pedal up and the engine above this
        // many times the idle speed, nothing fires until it comes back down (0: never)
        double idleFuelCutRatio = 1.5;
    };

public:
    DirectThrottleLinkage();
    virtual ~DirectThrottleLinkage();

    void initialize(const Parameters &params);

    virtual void setSpeedControl(double s);
    virtual void update(double dt, Engine *engine);
    virtual void setIdleControlEnabled(bool enabled) { m_idleControlEnabled = enabled; }

protected:
    double m_gamma;
    double m_throttlePosition;

    double m_idleSpeed = 0;
    double m_idleMaxOpening = 0.3;
    double m_idleOpening = 0;
    double m_idleProportional = 0.6;
    double m_idleIntegral = 0.5;
    double m_idleFuelCutRatio = 1.5;
    bool m_idleControlEnabled = true;
};

#endif /* ATG_ENGINE_SIM_DIRECT_THROTTLE_LINKAGE_H */
