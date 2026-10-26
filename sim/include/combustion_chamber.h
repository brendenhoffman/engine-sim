#ifndef ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H
#define ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H

#include "scs.h"

#include "piston.h"
#include "gas_system.h"
#include "cylinder_head.h"
#include "units.h"
#include "fuel.h"
#include "pipe.h"

#include <random>
#include <vector>

class Engine;
class CombustionChamber : public atg_scs::ForceGenerator {
    public:
        struct Parameters {
            Piston *piston;
            CylinderHead *Head;
            Fuel *fuel;
            Function *MeanPistonSpeedToTurbulence;

            double StartingPressure;
            double StartingTemperature;
            double CrankcasePressure;
            double HeatTransferCoefficient = 100.0;              // W/m^2/K
            double BlockTemperature = units::celcius(90.0);
            double HeatTransferReferenceSpeed = 0;               // See Engine::Parameters
            double PipeResolution = 0;                           // See Engine::Parameters
        };

        struct FlameEvent {
            double lit_n = 0;
            double total_n = 0;
            double percentageLit = 0;
            double efficiency = 1.0;
            double flameSpeed = 0.0;

            double lastVolume = 0.0;
            double travel_x = 0.0;
            double travel_y = 0.0;
            GasSystem::Mix globalMix;

            // Burned gas left in (or pulled back into) the charge, as a molar fraction:
            // fresh charge has inert gas and oxygen in air's 3:1 ratio, products add inert
            double residualFraction = 0.0;

            // Rotor faces: where the leading plug fired (cycle angle), for the trailing ones
            double ignitionAngle = 0.0;
            double undilutedFlameSpeed = 0.0; // Before residual gas and cyclic variation
            double equivalenceRatio = 0.0;
        };

        // How the last charge went, for diagnostics: how much of it the flame reached
        // before the next ignition (or the exhaust took it)
        double lastBurnCompletion() const { return m_lastBurnCompletion; }
        int misfires() const { return m_misfires; }

        struct FrictionModelParams {
            double frictionCoeff = 0.06;
            double breakawayFriction = units::force(50, units::N);
            double breakawayFrictionVelocity = units::distance(0.1, units::m);
            double viscousFrictionCoefficient = units::force(20, units::N);
        };

    public:
        CombustionChamber();
        virtual ~CombustionChamber();

        void initialize(const Parameters &params);
        void destroy();
        void setEngine(Engine *engine) { m_engine = engine; }
        virtual void apply(atg_scs::SystemState *system);

        CylinderHead *getCylinderHead() const { return m_head; }
        Piston *getPiston() const { return m_piston; }

        double getFrictionForce() const;
        double getVolume() const;
        double pistonSpeed() const;
        double calculateMeanPistonSpeed() const;
        double calculateFiringPressure() const;

        bool isLit() const { return m_lit; }

        // A Wankel rotor face rather than a cylinder: its volume follows the eccentric shaft
        // and it pushes the shaft directly
        bool isRotorFace() const { return m_rotorFace; }
        double rotorVolumeDerivative() const; // dV per radian of cycle angle
        bool popLitLastFrame();

        void ignite();
        void update(double dt);
        void flow(double dt);

        double lastEventAfr() const;

        double getLastIterationExhaustFlow() const { return m_exhaustFlow; }

        void resetLastTimestepExhaustFlow() { m_lastTimestepTotalExhaustFlow = 0; }
        double getLastTimestepExhaustFlow() const { return m_lastTimestepTotalExhaustFlow; }

        void resetLastTimestepIntakeFlow() { m_lastTimestepTotalIntakeFlow = 0; }
        double getLastTimestepIntakeFlow() const { return m_lastTimestepTotalIntakeFlow; }

        Function *m_meanPistonSpeedToTurbulence;
        GasSystem m_system;
        // The signal the exhaust contributes to the sound: pressure above ambient where the
        // primary meets the collector, plus a little of its flow
        double exhaustSoundSignal() const;
        double rawExhaustSoundSignal() const;
        // How far that signal still has to travel to the tailpipe
        double exhaustSoundPathLength() const;
        // The gas in the intake runner next to the port
        GasSystem::Mix intakeRunnerMix() const;
        bool hasPipes() const { return m_intakePipeRef != nullptr; }
        // The gas model for the chamber, its runners and the pipes it owns
        void setGasProperties(const GasSystem::Properties *properties) {
            m_system.setProperties(properties);
            m_intakeRunnerAndManifold.setProperties(properties);
            m_crankcase.setProperties(properties);
            m_exhaustRunnerAndPrimary.setProperties(properties);
            if (m_pipeOwner) {
                m_intakePipe.setProperties(properties);
                m_exhaustPipe.setProperties(properties);
            }
        }

        Pipe m_intakePipe;   // Plenum to intake valve, when the engine uses pipes
        Pipe m_exhaustPipe;  // Exhaust valve to collector
        // The pipes this chamber breathes through: its own, or for a rotor's second and third
        // faces, the first face's (one port and runner per housing, which the faces pass in turn)
        Pipe *m_intakePipeRef = nullptr;
        Pipe *m_exhaustPipeRef = nullptr;
        int m_intakePortConnection = 0;
        int m_exhaustPortConnection = 0;
        bool m_pipeOwner = false;

        GasSystem m_intakeRunnerAndManifold;
        GasSystem m_exhaustRunnerAndPrimary;
        FlameEvent m_flameEvent;
        bool m_lit;

        FrictionModelParams m_frictionModel;

        double m_peakTemperature;
        double m_nBurntFuel;

    protected:
        double calculateFrictionForce(double v) const;
        void updateCycleStates();

        double m_intakeFlowRate;
        double m_exhaustFlowRate;

        double m_manifoldToRunnerFlowRate;
        double m_primaryToCollectorFlowRate;
        double m_cylinderCrossSectionSurfaceArea;
        double m_cylinderWidthApproximation;

        double m_lastTimestepTotalExhaustFlow;
        double m_lastTimestepTotalIntakeFlow;
        double m_exhaustFlow;

        double m_crankcasePressure;
        bool m_rotorFace = false;
        double m_lastBurnCompletion = 0;

        // Diesel: this cycle's injection and how its burn is going
        struct DieselEvent {
            bool active = false;       // Injected, not yet burned out
            bool burning = false;      // Past the ignition delay
            double injected = 0;       // mol of fuel
            double burned = 0;         // mol the burn law has reached
            double delay = 0;          // Ignition delay integral (lights at 1)
            double elapsed = 0;        // s since injection
            double premixed = 0.5;     // Share burned in the premixed spike
            double startAngle = 0;     // Cycle angle it lit at
            double equivalence = 0;
        };
        DieselEvent m_diesel;
        double m_injectorBias = 0;     // This cylinder's injector, relative (0: not drawn yet)
        double m_delayScale = 1;       // This shot's ignition delay, relative
        void injectDiesel();
        void burnDiesel(double dt);

        // Two-stroke: the crankcase the piston's underside pumps, and the ports
    public:
        bool isTwoStroke() const;
        bool hasCrankcase() const;
        GasSystem m_crankcase;
        double crankcasePressure() const { return hasCrankcase() ? m_crankcase.pressure() : m_crankcasePressure; }
        double powerValveOpening() const { return m_powerValve; }
    protected:
        double m_transferFlowRate = 0;
        double m_reed = 0;              // Reed valve opening, 0-1
        double m_powerValve = 0;        // Power valve opening, 0-1
        double m_lastPistonTravel = 0;
        double pistonTravel() const;    // From TDC, m
        double travelAt(double angleFromTdc) const;
        double angleFromTdc(double travel, bool falling) const;
        void updateTwoStrokePorts(double dt);
        void transferFlow(double dt, double *transfer);

        // Jake brake: the exhaust valve held cracked open after compression
        bool m_jakeOpen = false;
        double m_jakeSound = 0;        // The exhaust signal low-passed, heard while the Jake works
        double m_jakeBlend = 0;
        double m_lastVolume = 0;
        double m_volumeMin = 1e9, m_volumeMax = 0;
        int m_misfires = 0;
        double m_heatTransferCoefficient = 100.0;
        double m_blockTemperature = units::celcius(90.0);
        double m_heatTransferReferenceSpeed = 0;
        double m_pistonSpeedSum = 0; // Running sum of m_pistonSpeed[], for the mean

        double *m_pressure;
        double *m_pistonSpeed;
        static constexpr int StateSamples = 256;

        bool m_litLastFrame;

        // Its own generator: the shared rand() is also used by the audio thread, which made
        // runs unrepeatable
        std::minstd_rand m_random;

        Piston *m_piston;
        CylinderHead *m_head;
        Engine *m_engine;
        Fuel *m_fuel;
};

#endif /* ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H */
