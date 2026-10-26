#ifndef ATG_ENGINE_SIM_FORCED_INDUCTION_NODE_H
#define ATG_ENGINE_SIM_FORCED_INDUCTION_NODE_H

#include "object_reference_node.h"

#include "exhaust_system_node.h"
#include "intake_node.h"

#include "engine_sim.h"

namespace es_script {

    // One turbocharger or supercharger
    class CompressorStageNode : public ObjectReferenceNode<CompressorStageNode> {
    public:
        CompressorStageNode() { /* void */ }
        virtual ~CompressorStageNode() { /* void */ }

        ForcedInduction::Stage stage() const {
            ForcedInduction::Stage s = m_stage;
            s.turbo = m_turbo > 0.5;
            s.feedsBlower = m_feedsBlower > 0.5;
            return s;
        }

    protected:
        virtual void registerInputs() {
            addInput("turbo", &m_turbo);
            addInput("compressor_radius", &m_stage.compressorRadius);
            addInput("compressor_flow", &m_stage.compressorFlow);
            addInput("compressor_efficiency", &m_stage.compressorEfficiency);
            addInput("slip", &m_stage.slip);
            addInput("duct_length", &m_stage.ductLength);
            addInput("duct_area", &m_stage.ductArea);
            addInput("turbine_radius", &m_stage.turbineRadius);
            addInput("turbine_flow", &m_stage.turbineFlow);
            addInput("turbine_efficiency", &m_stage.turbineEfficiency);
            addInput("inertia", &m_stage.inertia);
            addInput("max_speed", &m_stage.maxSpeed);
            addInput("friction", &m_stage.friction);
            addInput("drive_ratio", &m_stage.driveRatio);
            addInput("sequential_rpm", &m_stage.sequentialSpeed);
            addInput("prespool", &m_stage.prespool);
            addInput("feeds_blower", &m_feedsBlower);
            addInput("whine_order", &m_stage.whineOrder);
            addInput("gear_order", &m_stage.gearOrder);
            addInput("displacement", &m_stage.displacement);
            addInput("leakage", &m_stage.leakage);
            addInput("gear_volume", &m_stage.gearVolume);

            ObjectReferenceNode<CompressorStageNode>::registerInputs();
        }

        virtual void _evaluate() {
            setOutput(this);
            readAllInputs();
        }

        double m_turbo = 1;
        double m_feedsBlower = 0;
        ForcedInduction::Stage m_stage;
    };

    // The charge path between the compressors and an intake's throttle, its valves, and the
    // exhaust system feeding any turbines. Stages come from add_stage.
    class ForcedInductionNode : public ObjectReferenceNode<ForcedInductionNode> {
    public:
        ForcedInductionNode() { /* void */ }
        virtual ~ForcedInductionNode() { /* void */ }

        void addStage(const CompressorStageNode *stage) { m_setup.stages.push_back(stage->stage()); }

    protected:
        virtual void registerInputs() {
            addInput("intake", &m_intake, InputTarget::Type::Object);
            addInput("exhaust_system", &m_exhaust, InputTarget::Type::Object);
            addInput("charge_volume", &m_setup.parameters.chargeVolume);
            addInput("interstage_volume", &m_setup.parameters.interstageVolume);
            addInput("intercooler", &m_setup.parameters.intercooler);
            addInput("boost", &m_setup.parameters.boost);
            addInput("wastegate_flow", &m_setup.parameters.wastegateFlow);
            addInput("wastegate_range", &m_setup.parameters.wastegateRange);
            addInput("bov_flow", &m_setup.parameters.bovFlow);
            addInput("bov_pressure", &m_setup.parameters.bovPressure);
            addInput("bov_vent", &m_bovVent);
            addInput("whine_volume", &m_setup.parameters.whineVolume);
            addInput("whine_floor", &m_setup.parameters.whineFloor);
            addInput("hiss_volume", &m_setup.parameters.hissVolume);
            addInput("bov_volume", &m_setup.parameters.bovVolume);
            addInput("flutter_volume", &m_setup.parameters.flutterVolume);
            addInput("wastegate_volume", &m_setup.parameters.wastegateVolume);

            ObjectReferenceNode<ForcedInductionNode>::registerInputs();
        }

        virtual void _evaluate() {
            setOutput(this);
            readAllInputs();

            m_setup.parameters.bovVent = m_bovVent > 0.5;
            m_setup.exhaust = m_exhaust;
            m_intake->setForcedInduction(&m_setup);
        }

        IntakeNode *m_intake = nullptr;
        ExhaustSystemNode *m_exhaust = nullptr;
        double m_bovVent = 1;
        ForcedInductionSetup m_setup;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_FORCED_INDUCTION_NODE_H */
