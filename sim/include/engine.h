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

            // Mechanical friction as a mean effective pressure (Chen-Flynn form), applied to the
            // crank in place of the per-piston wall friction: FMEP = constant
            // + peakPressure x (peak cylinder pressure) + speed x Sp + speed2 x Sp^2, with Sp the
            // mean piston speed. Off: upstream's piston friction.
            struct Friction {
                bool enabled = false;
                double constant = 30e3;         // Pa
                double peakPressure = 0.006;
                double speed = 4000;            // Pa per m/s
                double speed2 = 120;            // Pa per (m/s)^2
            } friction;

            // Diesel fuel governor. The pedal sets the share of the pump's full delivery
            // (maxFuelMass per injection); a PI idle governor holds idleSpeed; above ratedSpeed
            // the fuel falls away to nothing over the droop (8%). Whatever is asked for, no more
            // is injected than the cylinder's oxygen could burn at maxEquivalence (the smoke
            // limit). With no maxFuelMass the share applies to that limit instead.
            struct Diesel {
                double idleSpeed = 0;           // rad/s
                double ratedSpeed = 0;          // rad/s (0: no limit but the ignition module's)
                double maxFuelMass = 0;         // kg per injection
                // The full-load curve: full delivery up to peakTorqueSpeed, falling in a line to
                // ratedFuel (a share of it) at ratedSpeed, as fuel maps shape a torque curve
                double peakTorqueSpeed = 0;     // rad/s (0: full delivery all the way)
                double ratedFuel = 1.0;
                double maxEquivalence = 0.75;
                double idleMaxEquivalence = 0.35; // The idle governor's most, as a share of full delivery
                double governorProportional = 2.0;
                double governorIntegral = 1.0;
            } diesel;

            // Compression-release engine brake: off throttle, the exhaust valve is cracked open
            // at the top of the compression stroke, dumping the compressed air (flow constant;
            // 0: none)
            double jakeBrakeFlow = 0;

            // The cylinder pressure's rise radiated straight from the block (diesel knock), mixed
            // in with the exhaust (0: none)
            double combustionNoise = 0;
            double knockResonance = 0;      // See Synthesizer::Parameters

            // Two-stroke ports (cycle_angle 360 deg). Ports in the cylinder wall open as the piston
            // uncovers them, so their timing (total degrees open, centred on BDC) sets how high
            // they sit; their area grows with how much is uncovered.
            struct TwoStroke {
                double exhaustPortDuration = 0;     // rad (0: exhaust through the head's valves)
                double exhaustPortFlow = 0;         // flow constant, fully open
                // Exhaust power valve: a shorter port (lower timing) at low revs, opening up to
                // the full port between these speeds (0: none)
                double exhaustPortDurationLow = 0;
                double powerValveStart = 0, powerValveEnd = 0;  // rad/s
                double transferPortDuration = 0;    // rad (0: not a two-stroke)
                double transferPortFlow = 0;
                // Crankcase scavenging: the piston's underside pumps the crankcase, which feeds
                // the transfers. Primary compression ratio (about 1.3-1.5; 0: a blower-fed airbox,
                // the transfers drawing from the intake)
                double crankcaseRatio = 0;
                // How charge gets into the crankcase: 0 a piston port (open centred on TDC for
                // inletDuration), 1 a reed valve (opens on the pressure across it), 2 a rotary
                // disc valve (open from discOpen to discClose, crank degrees after TDC)
                int inletType = 0;
                double inletDuration = 0;
                double discOpen = 0, discClose = 0;
                double inletFlow = 0;
                double reedPressure = 3000;         // Pa across it to open fully
                double reedResponse = 0.0008;       // s
                // How well the fresh charge pushes the burned gas out ahead of it rather than
                // mixing in (0: perfect mixing; about 1: a good loop-scavenged engine). The gas
                // leaving while the transfers flow carries fresh share f^(1 + this).
                double scavengingQuality = 1.0;
                // Expansion chamber (the exhaust pipe's shape; with pipe_resolution): header,
                // diffuser cone, belly, baffle cone, stinger (bellyDiameter 0: a plain pipe)
                double headerLength = 0, headerDiameter = 0, diffuserLength = 0;
                double bellyDiameter = 0, bellyLength = 0, baffleLength = 0;
                double stingerLength = 0, stingerDiameter = 0;
                bool enabled() const { return transferPortDuration > 0; }
            } twoStroke;
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
        const Parameters::Friction &getFriction() const { return m_friction; }
        double getStroke() const { return m_stroke; }   // Mean, over the piston cylinders
        const Parameters::Diesel &getDiesel() const { return m_diesel; }
        double getCombustionNoise() const { return m_combustionNoise; }
        double getKnockResonance() const { return m_knockResonance; }
        const Parameters::TwoStroke &getTwoStroke() const { return m_twoStroke; }
        double getJakeBrakeFlow() const { return m_jakeBrakeFlow; }

        // Diesel fueling this step (share of full delivery), and the governor that sets it
        double getDieselFueling() const { return m_dieselFueling; }
        void updateDieselGovernor(double dt);
        // Jake brake: switched on (a cab switch; on by default) and acting this step
        void setJakeBrakeEnabled(bool on) { m_jakeBrakeEnabled = on; }
        bool isJakeBrakeEnabled() const { return m_jakeBrakeEnabled; }
        bool isJakeBrakeActive() const { return m_jakeBrakeActive; }
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
        Parameters::Friction m_friction;
        Parameters::Diesel m_diesel;
        double m_combustionNoise = 0;
        double m_knockResonance = 0;
        Parameters::TwoStroke m_twoStroke;
        double m_jakeBrakeFlow = 0;
        double m_dieselFueling = 0;
        double m_dieselIntegral = 0;
        bool m_jakeBrakeEnabled = false;
        bool m_jakeBrakeActive = false;
        double m_stroke = 0;

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
