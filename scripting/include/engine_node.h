#ifndef ATG_ENGINE_SIM_ENGINE_NODE_H
#define ATG_ENGINE_SIM_ENGINE_NODE_H

#include "object_reference_node.h"

#include "crankshaft_node.h"
#include "cylinder_bank_node.h"
#include "ignition_module_node.h"
#include "engine_context.h"
#include "fuel_node.h"
#include "throttle_nodes.h"

#include "engine_sim.h"

#include <map>
#include <cmath>
#include <set>

namespace es_script {

    class EngineNode : public ObjectReferenceNode<EngineNode> {
    public:
        EngineNode() { /* void */ }
        virtual ~EngineNode() { /* void */ }

        void buildEngine(Engine *engine) {
            int cylinderCount = 0;
            for (const CylinderBankNode *bank : m_cylinderBanks) {
                cylinderCount += bank->getCylinderCount();
            }

            std::set<ExhaustSystemNode *> exhaustSystems;
            std::set<IntakeNode *> intakes;
            for (const CylinderBankNode *bank : m_cylinderBanks) {
                const int n = bank->getCylinderCount();
                for (int i = 0; i < n; ++i) {
                    exhaustSystems.insert(bank->getCylinder(i).exhaust);
                    intakes.insert(bank->getCylinder(i).intake);
                }
            }

            EngineContext context;
            context.setEngine(engine);

            Engine::Parameters parameters = m_parameters;
            parameters.idleFreeRunning = m_idleFreeRunning > 0.5;
            parameters.realGas = m_realGas > 0.5;
            parameters.friction.enabled = m_frictionModel > 0.5;
            parameters.twoStroke.inletType = static_cast<int>(std::lround(m_inletType));
            parameters.crankshaftCount = (int)m_crankshafts.size();
            parameters.cylinderBanks = (int)m_cylinderBanks.size();
            parameters.cylinderCount = cylinderCount;
            parameters.exhaustSystemCount = (int)exhaustSystems.size();
            parameters.intakeCount = (int)intakes.size();
            parameters.throttle = m_throttle->generate();
            engine->initialize(parameters);

            {
                int i = 0;
                for (ExhaustSystemNode *exhaust : exhaustSystems) {
                    context.addExhaust(
                        exhaust, engine->getExhaustSystem(i++));
                }
            }

            {
                int i = 0;
                for (IntakeNode *intake : intakes) {
                    context.addIntake(
                        intake, engine->getIntake(i++));
                }
            }

            {
                int i = 0;
                for (const CylinderBankNode *bank : m_cylinderBanks) {
                    context.addHead(bank->getCylinderHead(), engine->getHead(i++));
                }
            }

            for (const CylinderBankNode *bank : m_cylinderBanks) {
                const int n = bank->getCylinderCount();
                for (int i = 0; i < n; ++i) {
                    exhaustSystems.insert(bank->getCylinder(i).exhaust);
                    intakes.insert(bank->getCylinder(i).intake);
                }
            }

            for (int i = 0; i < parameters.crankshaftCount; ++i) {
                m_crankshafts[i]->generate(engine->getCrankshaft(i), &context);
                engine->getCrankshaft(i)->setCycleLength(parameters.cycleLength);
            }

            for (int i = 0; i < parameters.cylinderBanks; ++i) {
                m_cylinderBanks[i]->indexSlaveJournals(&context);
            }

            int cylinderIndex = 0;
            for (int i = 0; i < parameters.cylinderBanks; ++i) {
                m_cylinderBanks[i]->generate(
                    i,
                    cylinderIndex,
                    engine->getCylinderBank(i),
                    engine->getCrankshaft(0),
                    engine,
                    &context);
                cylinderIndex += m_cylinderBanks[i]->getCylinderCount();
            }

            for (int i = 0; i < parameters.cylinderBanks; ++i) {
                m_cylinderBanks[i]->connectRodAssemblies(&context);
            }

            m_ignitionModule->generate(engine, &context);
            
            Function *meanPistonSpeedToTurbulence = new Function;
            meanPistonSpeedToTurbulence->initialize(30, 1);
            for (int i = 0; i < 30; ++i) {
                const double s = (double)i;
                meanPistonSpeedToTurbulence->addSample(s, s * 0.5);
            }

            Fuel *fuel = engine->getFuel();
            m_fuel->generate(fuel, &context);

            CombustionChamber::Parameters ccParams;
            ccParams.CrankcasePressure = units::pressure(1.0, units::atm);
            ccParams.fuel = fuel;
            ccParams.StartingPressure = units::pressure(1.0, units::atm);
            ccParams.StartingTemperature = units::celcius(25.0);
            ccParams.MeanPistonSpeedToTurbulence = meanPistonSpeedToTurbulence;
            ccParams.HeatTransferCoefficient = parameters.heatTransferCoefficient;
            ccParams.BlockTemperature = parameters.blockTemperature;
            ccParams.HeatTransferReferenceSpeed = parameters.heatTransferReferenceSpeed;
            ccParams.PipeResolution = parameters.pipeResolution;

            for (int i = 0; i < engine->getCylinderCount(); ++i) {
                ccParams.piston = engine->getPiston(i);
                ccParams.Head = engine->getHead(ccParams.piston->getCylinderBank()->getIndex());
                engine->getChamber(i)->initialize(ccParams);
            }
        }

        void addCrankshaft(CrankshaftNode *crankshaft) {
            m_crankshafts.push_back(crankshaft);
        }

        void addCylinderBank(CylinderBankNode *bank) {
            m_cylinderBanks.push_back(bank);
        }

        int getIgnitionModuleCount() const {
            return m_ignitionModule == nullptr
                ? 0
                : 1;
        }

        void addIgnitionModule(IgnitionModuleNode *ignitionModule) {
            m_ignitionModule = ignitionModule;
        }

    protected:
        virtual void registerInputs() {
            addInput("name", &m_parameters.name);
            addInput("starter_torque", &m_parameters.starterTorque);
            addInput("starter_speed", &m_parameters.starterSpeed);
            addInput("dyno_min_speed", &m_parameters.dynoMinSpeed);
            addInput("dyno_max_speed", &m_parameters.dynoMaxSpeed);
            addInput("dyno_hold_step", &m_parameters.dynoHoldStep);
            addInput("redline", &m_parameters.redline);
            addInput("fuel", &m_fuel, InputTarget::Type::Object);
            addInput("throttle", &m_throttle, InputTarget::Type::Object);
            addInput("simulation_frequency", &m_parameters.initialSimulationFrequency);
            addInput("hf_gain", &m_parameters.initialHighFrequencyGain);
            addInput("jitter", &m_parameters.initialJitter);
            addInput("noise", &m_parameters.initialNoise);
            addInput("convolution", &m_parameters.initialConvolution);
            addInput("cycle_angle", &m_parameters.cycleLength);
            addInput("idle_rpm", &m_parameters.idleRpm);
            addInput("idle_free_running", &m_idleFreeRunning);
            addInput("fluid_simulation_steps", &m_parameters.fluidSimulationSteps);
            addInput("max_sle_solver_steps", &m_parameters.maxSleSolverSteps);
            addInput("heat_transfer_coefficient", &m_parameters.heatTransferCoefficient);
            addInput("block_temperature", &m_parameters.blockTemperature);
            addInput("heat_transfer_reference_speed", &m_parameters.heatTransferReferenceSpeed);
            addInput("pipe_resolution", &m_parameters.pipeResolution);
            addInput("real_gas", &m_realGas);
            addInput("friction_model", &m_frictionModel);
            addInput("fmep_constant", &m_parameters.friction.constant);
            addInput("fmep_peak_pressure", &m_parameters.friction.peakPressure);
            addInput("fmep_speed", &m_parameters.friction.speed);
            addInput("fmep_speed2", &m_parameters.friction.speed2);
            addInput("diesel_idle_speed", &m_parameters.diesel.idleSpeed);
            addInput("diesel_rated_speed", &m_parameters.diesel.ratedSpeed);
            addInput("diesel_max_fuel", &m_parameters.diesel.maxEquivalence);
            addInput("diesel_fuel_per_injection", &m_parameters.diesel.maxFuelMass);
            addInput("diesel_peak_torque_speed", &m_parameters.diesel.peakTorqueSpeed);
            addInput("diesel_rated_fuel", &m_parameters.diesel.ratedFuel);
            addInput("diesel_idle_max_fuel", &m_parameters.diesel.idleMaxEquivalence);
            addInput("diesel_governor_proportional", &m_parameters.diesel.governorProportional);
            addInput("diesel_governor_integral", &m_parameters.diesel.governorIntegral);
            addInput("jake_brake_flow", &m_parameters.jakeBrakeFlow);
            addInput("combustion_noise", &m_parameters.combustionNoise);
            addInput("knock_resonance", &m_parameters.knockResonance);
            addInput("exhaust_port_duration", &m_parameters.twoStroke.exhaustPortDuration);
            addInput("exhaust_port_flow", &m_parameters.twoStroke.exhaustPortFlow);
            addInput("exhaust_port_duration_low", &m_parameters.twoStroke.exhaustPortDurationLow);
            addInput("power_valve_start", &m_parameters.twoStroke.powerValveStart);
            addInput("power_valve_end", &m_parameters.twoStroke.powerValveEnd);
            addInput("transfer_port_duration", &m_parameters.twoStroke.transferPortDuration);
            addInput("transfer_port_flow", &m_parameters.twoStroke.transferPortFlow);
            addInput("crankcase_ratio", &m_parameters.twoStroke.crankcaseRatio);
            addInput("inlet_duration", &m_parameters.twoStroke.inletDuration);
            addInput("disc_open", &m_parameters.twoStroke.discOpen);
            addInput("disc_close", &m_parameters.twoStroke.discClose);
            addInput("inlet_flow", &m_parameters.twoStroke.inletFlow);
            addInput("reed_pressure", &m_parameters.twoStroke.reedPressure);
            addInput("reed_response", &m_parameters.twoStroke.reedResponse);
            addInput("scavenging_quality", &m_parameters.twoStroke.scavengingQuality);
            addInput("header_length", &m_parameters.twoStroke.headerLength);
            addInput("header_diameter", &m_parameters.twoStroke.headerDiameter);
            addInput("diffuser_length", &m_parameters.twoStroke.diffuserLength);
            addInput("belly_diameter", &m_parameters.twoStroke.bellyDiameter);
            addInput("belly_length", &m_parameters.twoStroke.bellyLength);
            addInput("baffle_length", &m_parameters.twoStroke.baffleLength);
            addInput("stinger_length", &m_parameters.twoStroke.stingerLength);
            addInput("stinger_diameter", &m_parameters.twoStroke.stingerDiameter);
            addInput("inlet_type", &m_inletType);

            ObjectReferenceNode<EngineNode>::registerInputs();
        }

        virtual void _evaluate() {
            setOutput(this);

            // Read inputs
            readAllInputs();
        }

        ThrottleNode *m_throttle = nullptr;
        IgnitionModuleNode *m_ignitionModule = nullptr;
        FuelNode *m_fuel = nullptr;

        Engine::Parameters m_parameters;
        double m_idleFreeRunning = 0;
        double m_realGas = 0;
        double m_frictionModel = 0;
        double m_inletType = 0;
        std::vector<CrankshaftNode *> m_crankshafts;
        std::vector<CylinderBankNode *> m_cylinderBanks;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_ENGINE_NODE_H */
