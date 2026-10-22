#ifndef ATG_ENGINE_SIM_FORCED_INDUCTION_H
#define ATG_ENGINE_SIM_FORCED_INDUCTION_H

#include "gas_system.h"

#include <vector>

class Engine;
class ExhaustSystem;
class Intake;

// Turbochargers and superchargers. One or more compressor stages fill a charge volume (the
// pipes and intercooler between the compressors and the throttle), which the intake's throttle
// draws from instead of the open air.
//
// A centrifugal compressor is modelled as Moore and Greitzer do: the gas in its duct has
// inertia, and the pressure rise it makes depends on the flow through it along a characteristic
// that peaks at the surge line. Left of the peak a throttled compressor is unstable: the flow
// collapses, reverses and recovers over and over, which is turbo flutter (and, with no blow-off
// valve, the chirp of a supercharged bike shutting its throttle). The characteristic scales with
// the square of the impeller's tip speed.
//
// A turbocharger's turbine is fed from an exhaust system's collector (taking over its outlet,
// in series with what's downstream). Its torque is the classic radial-turbine line, highest when
// stalled and falling with blade speed, peaking in efficiency at a blade speed ratio of 0.7. A
// wastegate bleeds exhaust past it to hold the boost. A supercharger's impeller is geared to the
// crankshaft instead, which pays for its work.
//
// Sequential twins: a stage with a sequential speed only gets a trickle of exhaust (prespool)
// below that engine speed, and a check valve keeps its compressor out until it can push.
class ForcedInduction {
    public:
        struct Stage {
            bool turbo = true;

            // Compressor
            double compressorRadius = 0.025;       // Impeller tip, m
            double compressorFlow = 0.25;          // Choke flow at max speed, kg/s
            double compressorEfficiency = 0.72;    // Peak isentropic efficiency
            double slip = 0.9;                     // Work factor (how much of U^2 it gives)
            double ductLength = 0.5;               // Inlet + outlet ducting: the gas's inertia
            double ductArea = 0.002;               // m^2

            // Turbine
            double turbineRadius = 0.025;
            double turbineFlow = 0;                // engine-sim flow constant
            double turbineEfficiency = 0.7;

            // Shaft
            double inertia = 3e-5;                 // kg m^2
            double maxSpeed = 2.0e4;               // rad/s
            double friction = 2e-6;                // N m per rad/s

            // Supercharger: impeller speed / crank speed
            double driveRatio = 0;

            // Positive displacement (Roots, twin-screw, Eaton TVS): air moved per rotor turn
            // (0: centrifugal). It pumps its displacement whatever the pressure, less what leaks
            // back past the rotors, and compresses it at its adiabatic efficiency.
            double displacement = 0;               // m^3 per rotor revolution
            double leakage = 0;                    // Effective leak area, m^2

            // Sequential: brought in above this engine speed (rad/s; 0: always in)
            double sequentialSpeed = 0;
            double prespool = 0.1;                 // Its turbine's share of flow until then

            // Sound
            double whineOrder = 4;                 // Whine pitch, in multiples of shaft speed
            double gearOrder = 0;                  // Supercharger gear whine, multiples of crank speed (0: none)
            double gearVolume = 0.5;
        };

        struct Parameters {
            ExhaustSystem *exhaust = nullptr;      // Feeds the turbines
            double chargeVolume = 0.004;           // m^3
            double intercooler = 0;                // Effectiveness, 0-1
            double boost = 0;                      // Wastegate target, Pa above ambient (0: none)
            double wastegateFlow = 0;              // engine-sim flow constant, fully open
            double wastegateRange = 40000;         // Pa from cracking to fully open
            double bovFlow = 0;                    // Blow-off valve (0: none)
            double bovPressure = 20000;            // Charge over plenum pressure that opens it
            bool bovVent = true;                   // Vents to the air (false: recirculates)

            double whineVolume = 1.0;
            double hissVolume = 1.0;
            double bovVolume = 1.0;
            double flutterVolume = 1.0;
            double wastegateVolume = 0.0;
        };

        struct StageState {
            double speed = 0;          // Shaft, rad/s
            double massFlow = 0;       // Through the compressor, kg/s (negative: backwards)
            double turbineFlow = 0;    // kg/s
            double compressorTorque = 0;
            double turbineTorque = 0;
            bool engaged = true;       // Sequential stage brought in
        };

        // What the synthesizer needs for each stage
        struct Voice {
            double frequency = 0;      // Whine pitch, Hz
            double whine = 0;          // Levels, about 0-1 (times the engine's volumes)
            double hiss = 0;           // Air rushing through the compressor
            double flutter = 0;        // Signed compressor flow: its swings are surge
            double bov = 0;
            double wastegate = 0;
            double gearFrequency = 0;  // A supercharger's drive gears
            double gear = 0;
        };

    public:
        void initialize(const Parameters &params, const std::vector<Stage> &stages, const GasSystem::Mix &mix);
        void setGasProperties(const GasSystem::Properties *properties) { m_charge.setProperties(properties); }

        // One fluid step: compressors fill the charge volume, turbines and wastegate drain the
        // exhaust collector, shafts spin up or down
        void process(double dt, Engine *engine, Intake *intake);

        // Load the superchargers put on the crankshaft (N m, against its rotation)
        double crankLoad() const { return m_crankLoad; }

        GasSystem *charge() { return &m_charge; }
        const GasSystem *charge() const { return &m_charge; }
        ExhaustSystem *exhaust() const { return m_params.exhaust; }
        int stageCount() const { return static_cast<int>(m_stages.size()); }
        const Stage &stage(int i) const { return m_stages[i]; }
        const StageState &state(int i) const { return m_state[i]; }
        Voice voice(int i) const;
        double boost() const;           // Charge pressure above ambient, Pa
        double wastegateOpening() const { return m_wastegate; }
        double bovOpening() const { return m_bov; }
        const Parameters &parameters() const { return m_params; }

        static constexpr double Ambient = 101325.0;

        // The compressor's pressure rise at a mass flow and shaft speed (Pa), and the work it
        // does on each kg (J/kg)
        double compressorRise(const Stage &s, double speed, double massFlow, double *work) const;

    private:
        Parameters m_params;
        std::vector<Stage> m_stages;
        std::vector<StageState> m_state;
        GasSystem m_charge;
        GasSystem::Mix m_freshMix;
        double m_wastegate = 0;
        double m_bov = 0;
        double m_bovFlow = 0;        // kg/s
        double m_wastegateFlow = 0;  // kg/s
        double m_crankLoad = 0;
        double m_bovReference = 1;   // kg/s a fully open valve passes on 1 bar
        double m_wastegateReference = 1;
};

#endif /* ATG_ENGINE_SIM_FORCED_INDUCTION_H */
