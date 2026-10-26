#ifndef ATG_ENGINE_SIM_EXHAUST_SYSTEM_H
#define ATG_ENGINE_SIM_EXHAUST_SYSTEM_H

#include "part.h"

#include "gas_system.h"
#include "impulse_response.h"

class ExhaustSystem : public Part {
    friend class Engine;

    public:
        struct Parameters {
            double length;
            double collectorCrossSectionArea;
            double outletFlowRate;
            double primaryTubeLength;
            double primaryFlowRate;
            double velocityDecay;
            double audioVolume;
            ImpulseResponse *impulseResponse;
            // Hear each cylinder's pulses against the manifold's pressure, not ambient (for
            // turbo diesels, whose weak blowdowns drown in the turbo's back pressure)
            double manifoldRelativeSound = 0; // 0-1: how much of the manifold's pressure to take off
            double manifoldRelativeSoundIdle = -1; // ...at no load (a diesel's idle); -1: the same
            // A muffler's drone: the sound low-passed at this many times the firing frequency,
            // following the revs (0: off). Leaves the firing pulse's fundamental to dominate.
            double droneFilter = 0;
        };

    public:
        ExhaustSystem();
        virtual ~ExhaustSystem();

        void initialize(const Parameters &params);
        virtual void destroy();

        void process(double dt);

        inline int getIndex() const { return m_index; }
        inline double getLength() const { return m_length; }
        inline double getFlow() const { return m_flow; }
        inline double getAudioVolume() const { return m_audioVolume; }
        inline double getPrimaryFlowRate() const { return m_primaryFlowRate; }
        inline double getCollectorCrossSectionArea() const { return m_collectorCrossSectionArea; }
        inline double getPrimaryTubeLength() const { return m_primaryTubeLength; }
        inline double getVelocityDecay() const { return m_velocityDecay; }
        inline ImpulseResponse *getImpulseResponse() const { return m_impulseResponse; }

        inline GasSystem *getSystem() { return &m_system; }
        inline double getOutletFlowRate() const { return m_outletFlowRate; }
        // A turbocharger's turbine (and wastegate) takes over the outlet
        void setOutletExternal(bool external) { m_outletExternal = external; }
        bool isOutletExternal() const { return m_outletExternal; }
        double getManifoldRelativeSound() const { return m_manifoldRelativeSound; }
        double getDroneFilter() const { return m_droneFilter; }
        // The weight at a load (0-1)
        double manifoldRelativeSoundAt(double load) const {
            if (m_manifoldRelativeSoundIdle < 0) return m_manifoldRelativeSound;
            return m_manifoldRelativeSoundIdle + (m_manifoldRelativeSound - m_manifoldRelativeSoundIdle) * load;
        }
        // The manifold's pressure, lightly smoothed (see process())
        double getSmoothedPressure() const { return m_smoothedPressure; }
        void setExternalFlow(double flow) { m_flow = flow; }
        void setGasProperties(const GasSystem::Properties *properties) {
            m_system.setProperties(properties);
            m_atmosphere.setProperties(properties);
        }

    protected:
        GasSystem m_atmosphere;
        GasSystem m_system;

        ImpulseResponse *m_impulseResponse;

        double m_length;
        double m_primaryTubeLength;
        double m_collectorCrossSectionArea;
        double m_primaryFlowRate;
        double m_outletFlowRate;
        bool m_outletExternal = false;
        double m_manifoldRelativeSound = 0;
        double m_manifoldRelativeSoundIdle = -1;
        double m_droneFilter = 0;
        double m_smoothedPressure = 101325;
        double m_audioVolume;
        double m_velocityDecay;
        int m_index;

        double m_flow;
};

#endif /* ATG_ENGINE_SIM_EXHAUST_SYSTEM_H */
