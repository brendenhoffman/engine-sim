#ifndef ATG_ENGINE_SIM_FUEL_NODE_H
#define ATG_ENGINE_SIM_FUEL_NODE_H

#include "object_reference_node.h"

#include "engine_context.h"
#include "function_node.h"

#include "engine_sim.h"

#include <map>
#include <vector>

namespace es_script {

    class FuelNode : public ObjectReferenceNode<FuelNode> {
    public:
        FuelNode() { /* void */ }
        virtual ~FuelNode() { /* void */ }

        void generate(Fuel *fuel, EngineContext *context) const {
            Fuel::Parameters params = m_parameters;
            params.compressionIgnition = m_compressionIgnition > 0.5;
            params.turbulenceToFlameSpeedRatio =
                m_turbulenceToFlameSpeedRatio->generate(context);

            fuel->initialize(params);
        }

    protected:
        virtual void registerInputs() {
            addInput(
                "turbulence_to_flame_speed_ratio",
                &m_turbulenceToFlameSpeedRatio,
                InputTarget::Type::Object);
            addInput("name", &m_parameters.name);
            addInput("molecular_mass", &m_parameters.molecularMass);
            addInput("energy_density", &m_parameters.energyDensity);
            addInput("density", &m_parameters.density);
            addInput("molecular_afr", &m_parameters.molecularAfr);
            addInput("max_burning_efficiency", &m_parameters.maxBurningEfficiency);
            addInput("burning_efficiency_randomness", &m_parameters.burningEfficiencyRandomness);
            addInput("low_efficiency_attenuation", &m_parameters.lowEfficiencyAttenuation);
            addInput("max_turbulence_effect", &m_parameters.maxTurbulenceEffect);
            addInput("max_dilution_effect", &m_parameters.maxDilutionEffect);
            addInput("dilution_sensitivity", &m_parameters.dilutionSensitivity);
            addInput("cyclic_variation", &m_parameters.cyclicVariation);
            addInput("compression_ignition", &m_compressionIgnition);
            addInput("cetane_number", &m_parameters.cetaneNumber);
            addInput("injector_spread", &m_parameters.injectorSpread);
            addInput("misfire_residual", &m_parameters.misfireResidual);
            addInput("misfire_spread", &m_parameters.misfireSpread);
            addInput("shot_variation", &m_parameters.shotVariation);
            addInput("idle_shot_variation", &m_parameters.idleShotVariation);
            addInput("delay_variation", &m_parameters.delayVariation);

            ObjectReferenceNode<FuelNode>::registerInputs();
        }

        virtual void _evaluate() {
            setOutput(this);

            // Read inputs
            readAllInputs();
        }

        FunctionNode *m_turbulenceToFlameSpeedRatio = nullptr;
        double m_compressionIgnition = 0;
        Fuel::Parameters m_parameters;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_FUEL_NODE_H */
