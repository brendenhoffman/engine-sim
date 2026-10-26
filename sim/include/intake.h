#ifndef ATG_ENGINE_SIM_INTAKE_H
#define ATG_ENGINE_SIM_INTAKE_H

#include "part.h"

#include "gas_system.h"

class ForcedInduction;

class Intake : public Part {
    public:
        struct Parameters {
            // Plenum volume
            double volume;

            // Plenum dimensions
            double CrossSectionArea;

            // Input flow constant
            double InputFlowK;

            // Idle-circuit flow constant
            double IdleFlowK;

            // Flow rate from plenum to runner
            double RunnerFlowRate;

            // Molecular air fuel ratio (defaults to ideal for octane)
            double MolecularAfr = (25.0 / 2.0);

            // Throttle plate position at idle
            double IdleThrottlePlatePosition = 0.975;

            // Runner volume
            double RunnerLength = units::distance(4.0, units::inch);

            // Velocity decay factor
            double VelocityDecay = 0.5;
        };

    public:
        Intake();
        virtual ~Intake();

        void initialize(Parameters &params);
        virtual void destroy();

        void process(double dt);

        // The fuel-air mixture the carburettor makes
        GasSystem::Mix freshMix() const;

        inline double getRunnerFlowRate() const { return m_runnerFlowRate; }
        inline double getThrottlePlatePosition() const { return m_idleThrottlePlatePosition * m_throttle; }

        // Idle air: how far the plate closes at zero throttle (1 seals it). Adjustable live, like
        // an idle screw; the engine's own setting is kept for resetting.
        inline double getIdleThrottlePlatePosition() const { return m_idleThrottlePlatePosition; }
        inline void setIdleThrottlePlatePosition(double position) { m_idleThrottlePlatePosition = position; }
        inline double getDefaultIdleThrottlePlatePosition() const { return m_defaultIdleThrottlePlatePosition; }
        inline double getRunnerLength() const { return m_runnerLength; }
        inline double getPlenumCrossSectionArea() const { return m_crossSectionArea; }
        inline double getVelocityDecay() const { return m_velocityDecay; }

        void setGasProperties(const GasSystem::Properties *properties);

        // Turbochargers or superchargers feeding the throttle (owned; null: none)
        void setForcedInduction(ForcedInduction *forcedInduction);
        ForcedInduction *getForcedInduction() const { return m_forcedInduction; }

        // Diesels inject their fuel into the cylinders: the intake carries plain air
        void setDirectInjection(bool on) { m_directInjection = on; }

        GasSystem m_system;
        double m_throttle;

        double m_flow;
        double m_flowRate;
        double m_totalFuelInjected;

    protected:
        double m_crossSectionArea;
        double m_inputFlowK;
        double m_idleFlowK;
        double m_runnerFlowRate;
        double m_molecularAfr;
        double m_idleThrottlePlatePosition;
        double m_defaultIdleThrottlePlatePosition = 0;
        double m_runnerLength;
        double m_velocityDecay;

        GasSystem m_atmosphere;
        ForcedInduction *m_forcedInduction = nullptr;
        bool m_directInjection = false;
};

#endif /* ATG_ENGINE_SIM_INTAKE_H */
