#ifndef ATG_ENGINE_SIM_THROTTLE_H
#define ATG_ENGINE_SIM_THROTTLE_H

#include "part.h"

class Engine;
class Throttle {
public:
    Throttle();
    virtual ~Throttle();

    virtual void setSpeedControl(double s);
    virtual void update(double dt, Engine *engine);

    inline double getSpeedControl() const { return m_speedControl; }

    // Automatic idle control, where the throttle has one. Off while something else (the
    // BeamNG bridge) sets the engine's speed.
    virtual void setIdleControlEnabled(bool enabled) { (void)enabled; }

protected:
    double m_speedControl;
};

#endif /* ATG_ENGINE_SIM_THROTTLE_H */
