#ifndef ATG_ENGINE_SIM_ENGINE_H
#define ATG_ENGINE_SIM_ENGINE_H

#include "part.h"

#include "piston.h"
#include "connecting_rod.h"
#include "crankshaft.h"
#include "cylinder_bank.h"
#include "cylinder_head.h"
#include "exhaust_system.h"
#include "ignition_module.h"
#include "intake.h"
#include "combustion_chamber.h"
#include "units.h"
#include "throttle.h"

#include <string>

class Simulator;
class Vehicle;
class Transmission;
class Engine : public Part {
    public:
        struct Parameters {
            int cylinderBanks;
            int cylinderCount;
            int crankshaftCount;
            int exhaustSystemCount;
            int intakeCount;

            std::string name;

            double starterTorque = units::torque(90.0, units::ft_lb);
            double starterSpeed = units::rpm(200);
            double redline = units::rpm(6500);
            double dynoMinSpeed = units::rpm(1000);
            double dynoMaxSpeed = units::rpm(6500);
            double dynoHoldStep = units::rpm(100);

            Throttle *throttle;

            double initialSimulationFrequency;
            double initialHighFrequencyGain;
            double initialNoise;
            double initialJitter;
            double initialConvolution = 1.0;

            // The speed this engine is meant to idle at (0: unknown). Tells BeamNG where to
            // idle a car tuned to this engine; it doesn't control anything in engine-sim.
            double idleRpm = 0;
            // In BeamNG, idle freely as on its own (its idle control and fuel cut) rather than
            // following the game's idle speed: for lumpy big-overlap idles
            bool idleFreeRunning = false;

            // Crank angle of one engine cycle (see Crankshaft::setCycleLength)
            double cycleLength = 4 * constants::pi;

            // Performance: fluid substeps per simulation step, and the constraint solver's
            // iteration cap. Fewer is faster and less stable/accurate.
            double fluidSimulationSteps = 8;
            double maxSleSolverSteps = 128;

            // Charge heating: heat flow between the gas and the cylinder walls (W/m^2/K), and
            // the wall temperature (K)
            double heatTransferCoefficient = 100.0;
            double blockTemperature = units::celcius(90.0);
            // When set, the coefficient above holds at this mean piston speed and scales with
            // speed^0.8 (as measured heat transfer does): less loss at idle, more at high rpm.
            // 0 keeps it constant (upstream).
            double heatTransferReferenceSpeed = 0;

            // Intake runners and exhaust primaries as 1-D pipes with this many cells per metre,
            // so pressure waves travel in them (0: lumped volumes, upstream)
            double pipeResolution = 0;
            bool realGas = false;           // GasSystem::Properties::Real everywhere
        };

    public:
        Engine();
        virtual ~Engine();

        void initialize(const Parameters &params);
        virtual void destroy();

        std::string getName() const { return m_name; }

        virtual Crankshaft *getOutputCrankshaft() const;
        virtual void setSpeedControl(double s);
        virtual double getSpeedControl();
        virtual void setThrottle(double throttle);
        virtual double getThrottle() const;
        void setIdleControlEnabled(bool enabled); // The throttle's idle air control, if it has one
        virtual double getThrottlePlateAngle() const;
        virtual void calculateDisplacement();
        double getDisplacement() const { return m_displacement; }
        virtual double getIntakeFlowRate() const;
        virtual void update(double dt);

        virtual double getManifoldPressure() const;
        virtual double getIntakeAfr() const;
        virtual double getExhaustO2() const;
        virtual double getRpm() const;
        virtual double getSpeed() const;
        virtual bool isSpinningCw() const;

        virtual void resetFuelConsumption();
        virtual double getTotalFuelMassConsumed() const;
        double getTotalVolumeFuelConsumed() const;

        inline double getStarterTorque() const { return m_starterTorque; }
        inline double getStarterSpeed() const { return m_starterSpeed; }
        inline double getRedline() const { return m_redline; }
        inline double getDynoMinSpeed() const { return m_dynoMinSpeed; }
        inline double getDynoMaxSpeed() const { return m_dynoMaxSpeed; }
        inline double getDynoHoldStep() const { return m_dynoHoldStep; }

        int getCylinderBankCount() const { return m_cylinderBankCount; }
        int getCylinderCount() const { return m_cylinderCount; }
        int getCrankshaftCount() const { return m_crankshaftCount; }
        int getExhaustSystemCount() const { return m_exhaustSystemCount; }
        int getIntakeCount() const { return m_intakeCount; }
        int getMaxDepth() const;

        Crankshaft *getCrankshaft(int i) const { return &m_crankshafts[i]; }
        CylinderBank *getCylinderBank(int i) const { return &m_cylinderBanks[i]; }
        CylinderHead *getHead(int i) const { return &m_heads[i]; }
        Piston *getPiston(int i) const { return &m_pistons[i]; }
        ConnectingRod *getConnectingRod(int i) const { return &m_connectingRods[i]; }
        IgnitionModule *getIgnitionModule() { return &m_ignitionModule; }
        ExhaustSystem *getExhaustSystem(int i) const { return &m_exhaustSystems[i]; }
        Intake *getIntake(int i) const { return &m_intakes[i]; }
        CombustionChamber *getChamber(int i) const { return &m_combustionChambers[i]; }
        Fuel *getFuel() { return &m_fuel; }

        double getSimulationFrequency() const { return m_initialSimulationFrequency; }
        double getInitialHighFrequencyGain() const { return m_initialHighFrequencyGain; }
        double getInitialNoise() const { return m_initialNoise; }
        double getInitialJitter() const { return m_initialJitter; }
        double getInitialConvolution() const { return m_initialConvolution; }
        double getIdleRpm() const { return m_idleRpm; } // rad/s, 0 if not given
        bool getIdleFreeRunning() const { return m_idleFreeRunning; }
        int getFluidSimulationSteps() const { return m_fluidSimulationSteps; }
        int getMaxSleSolverSteps() const { return m_maxSleSolverSteps; }
        double getHeatTransferCoefficient() const { return m_heatTransferCoefficient; }
        double getBlockTemperature() const { return m_blockTemperature; }
        double getHeatTransferReferenceSpeed() const { return m_heatTransferReferenceSpeed; }
        double getPipeResolution() const { return m_pipeResolution; }
        bool getRealGas() const { return m_realGas; }
        // Turbochargers and superchargers across all intakes
        int getCompressorStageCount() const;

        virtual Simulator *createSimulator(Vehicle *vehicle, Transmission *transmission);

    protected:
        std::string m_name;

        Crankshaft *m_crankshafts;
        int m_crankshaftCount;

        CylinderBank *m_cylinderBanks;
        CylinderHead *m_heads;
        int m_cylinderBankCount;

        Piston *m_pistons;
        ConnectingRod *m_connectingRods;
        CombustionChamber *m_combustionChambers;
        int m_cylinderCount;

        double m_starterTorque;
        double m_starterSpeed;
        double m_redline;
        double m_dynoMinSpeed;
        double m_dynoMaxSpeed;
        double m_dynoHoldStep;

        double m_initialSimulationFrequency;
        double m_initialHighFrequencyGain;
        double m_initialNoise;
        double m_initialJitter;
        double m_initialConvolution = 1.0;
        double m_idleRpm = 0;
        bool m_idleFreeRunning = false;
        int m_fluidSimulationSteps = 8;
        int m_maxSleSolverSteps = 128;
        double m_heatTransferCoefficient = 100.0;
        double m_blockTemperature = 0;
        double m_heatTransferReferenceSpeed = 0;
        double m_pipeResolution = 0;
        bool m_realGas = false;

        ExhaustSystem *m_exhaustSystems;
        int m_exhaustSystemCount;

        Intake *m_intakes;
        int m_intakeCount;

        IgnitionModule m_ignitionModule;
        Fuel m_fuel;

        Throttle *m_throttle;

        double m_throttleValue;
        double m_displacement;
};

#endif /* ATG_ENGINE_SIM_ENGINE_H */
