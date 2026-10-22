#ifndef ATG_ENGINE_SIM_ACTIONS_H
#define ATG_ENGINE_SIM_ACTIONS_H

#include "piranha.h"

#include "compiler.h"
#include "object_reference_node.h"
#include "engine_node.h"
#include "crankshaft_node.h"
#include "rod_journal_node.h"
#include "cylinder_bank_node.h"
#include "function_node.h"
#include "camshaft_node.h"
#include "cylinder_head_node.h"
#include "cylinder_bank_node.h"
#include "ignition_module_node.h"
#include "transmission_node.h"
#include "vehicle_node.h"
#include "forced_induction_node.h"

namespace es_script {

    class SetEngineNode : public piranha::Node {
    public:
        SetEngineNode() { /* void */ }
        virtual ~SetEngineNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            registerInput(&m_engineInput, "engine");
        }

        virtual void _evaluate() {
            EngineNode *engineNode = getObject<EngineNode>(m_engineInput);

            Engine *engine = new Engine;
            engineNode->buildEngine(engine);
            Compiler::output()->engine = engine;
        }

    protected:
        piranha::pNodeInput m_engineInput = nullptr;
    };

    class AddRodJournalNode : public Node {
    public:
        AddRodJournalNode() { /* void */ }
        virtual ~AddRodJournalNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("crankshaft", &m_crankshaft, InputTarget::Type::Object);
            addInput("rod_journal", &m_rodJournal, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            m_crankshaft->addRodJournal(m_rodJournal);
        }

    protected:
        CrankshaftNode *m_crankshaft = nullptr;
        RodJournalNode *m_rodJournal = nullptr;
    };

    class AddSlaveJournalNode : public Node {
    public:
        AddSlaveJournalNode() { /* void */ }
        virtual ~AddSlaveJournalNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("rod", &m_rod, InputTarget::Type::Object);
            addInput("rod_journal", &m_rodJournal, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            m_rod->addRodJournal(m_rodJournal);
        }

    protected:
        ConnectingRodNode *m_rod = nullptr;
        RodJournalNode *m_rodJournal = nullptr;
    };

    class AddCrankshaftNode : public Node {
    public:
        AddCrankshaftNode() { /* void */ }
        virtual ~AddCrankshaftNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("engine", &m_engine, InputTarget::Type::Object);
            addInput("crankshaft", &m_crankshaft, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            m_engine->addCrankshaft(m_crankshaft);
        }

    protected:
        CrankshaftNode *m_crankshaft = nullptr;
        EngineNode *m_engine = nullptr;
    };

    class AddCylinderBankNode : public Node {
    public:
        AddCylinderBankNode() { /* void */ }
        virtual ~AddCylinderBankNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("engine", &m_engine, InputTarget::Type::Object);
            addInput("cylinder_bank", &m_cylinderBankNode, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            m_engine->addCylinderBank(m_cylinderBankNode);
        }

    protected:
        CylinderBankNode *m_cylinderBankNode = nullptr;
        EngineNode *m_engine = nullptr;
    };

    class AddRotorFaceNode : public Node {
    public:
        AddRotorFaceNode() { /* void */ }
        virtual ~AddRotorFaceNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("exhaust_system", &m_exhaustSystem, InputTarget::Type::Object);
            addInput("intake", &m_intake, InputTarget::Type::Object);
            addInput("rotor_housing", &m_cylinderBank, InputTarget::Type::Object);
            addInput("ignition_wire", &m_ignitionWire, InputTarget::Type::Object);
            addInput("primary_length", &m_primaryLength);
            addInput("sound_attenuation", &m_soundAttenuation);
            addInput("seal_leakage", &m_sealLeakage);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            m_cylinderBank->addRotorFace(
                m_intake,
                m_exhaustSystem,
                m_ignitionWire,
                m_soundAttenuation,
                m_primaryLength,
                m_sealLeakage);
        }

    protected:
        CylinderBankNode *m_cylinderBank = nullptr;
        ExhaustSystemNode *m_exhaustSystem = nullptr;
        IntakeNode *m_intake = nullptr;
        IgnitionWireNode *m_ignitionWire = nullptr;
        double m_primaryLength = 0.0;
        double m_soundAttenuation = 1.0;
        double m_sealLeakage = 0.0;
    };

    class AddCylinderNode : public Node {
    public:
        AddCylinderNode() { /* void */ }
        virtual ~AddCylinderNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("piston", &m_pistonNode, InputTarget::Type::Object);
            addInput("connecting_rod", &m_connectingRod, InputTarget::Type::Object);
            addInput("rod_journal", &m_rodJournal, InputTarget::Type::Object);
            addInput("exhaust_system", &m_exhaustSystem, InputTarget::Type::Object);
            addInput("intake", &m_intake, InputTarget::Type::Object);
            addInput("cylinder_bank", &m_cylinderBank, InputTarget::Type::Object);
            addInput("ignition_wire", &m_ignitionWire, InputTarget::Type::Object);
            addInput("primary_length", &m_primaryLength);
            addInput("sound_attenuation", &m_soundAttenuation);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            m_cylinderBank->addCylinder(
                m_pistonNode,
                m_connectingRod,
                m_rodJournal,
                m_intake,
                m_exhaustSystem,
                m_ignitionWire,
                m_soundAttenuation,
                m_primaryLength
            );
        }

    protected:
        PistonNode *m_pistonNode = nullptr;
        ConnectingRodNode *m_connectingRod = nullptr;
        RodJournalNode *m_rodJournal = nullptr;
        CylinderBankNode *m_cylinderBank = nullptr;
        ExhaustSystemNode *m_exhaustSystem = nullptr;
        IntakeNode *m_intake = nullptr;
        IgnitionWireNode *m_ignitionWire = nullptr;
        double m_primaryLength = 0.0;
        double m_soundAttenuation = 1.0;
    };

    class AddStageNode : public Node {
    public:
        AddStageNode() { /* void */ }
        virtual ~AddStageNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("stage", &m_stage, InputTarget::Type::Object);
            addInput("forced_induction", &m_forcedInduction, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();
            m_forcedInduction->addStage(m_stage);
        }

    protected:
        CompressorStageNode *m_stage = nullptr;
        ForcedInductionNode *m_forcedInduction = nullptr;
    };

    class AddSampleNode : public Node {
    public:
        AddSampleNode() { /* void */ }
        virtual ~AddSampleNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("x", &m_x);
            addInput("y", &m_y);
            addInput("function", &m_function, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            m_function->addSample(m_x, m_y);
        }

    protected:
        double m_x = 0;
        double m_y = 0;
        FunctionNode *m_function = nullptr;
    };

    class AddLobeNode : public Node {
    public:
        AddLobeNode() { /* void */ }
        virtual ~AddLobeNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("centerline", &m_centerline);
            addInput("camshaft", &m_camshaft, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            m_camshaft->addLobe(m_centerline);
        }

    protected:
        double m_centerline = 0;
        CamshaftNode *m_camshaft = nullptr;
    };

    class SetCylinderHeadNode : public Node {
    public:
        SetCylinderHeadNode() { /* void */ }
        virtual ~SetCylinderHeadNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("head", &m_head, InputTarget::Type::Object);
            addInput("bank", &m_bank, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            if (m_bank->getCylinderHead() != nullptr) {
                throwError("Cylinder bank already has a head");
                return;
            }
            else if (m_head->getBank() != nullptr) {
                throwError("Cylinder head already attached to a bank");
                return;
            }

            m_bank->setCylinderHead(m_head);
            m_head->setBank(m_bank);
        }

    protected:
        CylinderHeadNode *m_head = nullptr;
        CylinderBankNode *m_bank = nullptr;
    };

    template <int TestInches>
    class k_inH2ONode : public Node {
        class k_inH2ONodeOutput : public piranha::NodeOutput {
        public:
            k_inH2ONodeOutput() : NodeOutput(&piranha::FundamentalType::FloatType) {
                m_input = 0.0;
            }

            virtual ~k_inH2ONodeOutput() {
                /* void */
            }

            virtual void fullCompute(void *target) const {
                *reinterpret_cast<double *>(target) = GasSystem::k_inH2O(m_input, TestInches);
            }

            double getInput() const { return m_input; }
            void setInput(double data) { m_input = data; }

        protected:
            double m_input;
        };

    public:
        k_inH2ONode() { /* void */ }
        virtual ~k_inH2ONode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("flow", &m_flowInput);

            Node::registerInputs();
        }

        virtual void registerOutputs() {
            registerOutput(&m_output, "__out");

            setPrimaryOutput("__out");
        }

        virtual void _evaluate() {
            readAllInputs();

            m_output.setInput(m_flowInput);
        }

    protected:
        k_inH2ONodeOutput m_output;
        double m_flowInput = 0.0;
    };

    // Flow bench ratings at 28, 30 and 32 inches of water
    using k_28inH2ONode = k_inH2ONode<28>;
    using k_30inH2ONode = k_inH2ONode<30>;
    using k_32inH2ONode = k_inH2ONode<32>;

    class k_CarbNode : public Node {
        class k_CarbNodeOutput : public piranha::NodeOutput {
        public:
            k_CarbNodeOutput() : NodeOutput(&piranha::FundamentalType::FloatType) {
                m_input = 0.0;
            }

            virtual ~k_CarbNodeOutput() {
                /* void */
            }

            virtual void fullCompute(void *target) const {
                *reinterpret_cast<double *>(target) = GasSystem::k_carb(m_input);
            }

            double getInput() const { return m_input; }
            void setInput(double data) { m_input = data; }

        protected:
            double m_input;
        };

    public:
        k_CarbNode() { /* void */ }
        virtual ~k_CarbNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("flow", &m_flowInput);

            Node::registerInputs();
        }

        virtual void registerOutputs() {
            registerOutput(&m_output, "__out");

            setPrimaryOutput("__out");
        }

        virtual void _evaluate() {
            readAllInputs();

            m_output.setInput(m_flowInput);
        }

    protected:
        k_CarbNodeOutput m_output;
        double m_flowInput = 0.0;
    };

    class ConnectIgnitionWireNode : public Node {
    public:
        ConnectIgnitionWireNode() { /* void */ }
        virtual ~ConnectIgnitionWireNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("wire", &m_wire, InputTarget::Type::Object);
            addInput("ignition_module", &m_module, InputTarget::Type::Object);
            addInput("angle", &m_angle);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            m_module->connect(m_wire, m_angle);
        }

    protected:
        IgnitionWireNode *m_wire = nullptr;
        IgnitionModuleNode *m_module = nullptr;
        double m_angle = 0.0;
    };

    class AddIgnitionModuleNode : public Node {
    public:
        AddIgnitionModuleNode() { /* void */ }
        virtual ~AddIgnitionModuleNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("engine", &m_engine, InputTarget::Type::Object);
            addInput("ignition_module", &m_ignitionModule, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            m_engine->addIgnitionModule(m_ignitionModule);
        }

    protected:
        IgnitionModuleNode *m_ignitionModule = nullptr;
        EngineNode *m_engine = nullptr;
    };

    class GenerateHarmonicCamLobeNode : public Node {
    public:
        GenerateHarmonicCamLobeNode() { /* void */ }
        virtual ~GenerateHarmonicCamLobeNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("duration_at_50_thou", &m_durationAt50Thou);
            addInput("gamma", &m_gamma);
            addInput("lift", &m_lift);
            addInput("steps", &m_steps);
            addInput("cycle_angle", &m_cycleAngle);
            addInput("function", &m_function, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            // Half the duration, from crank degrees to cam angle (the cam turns once per cycle)
            const double angle = m_durationAt50Thou * constants::pi / m_cycleAngle;
            const double s = std::pow(2 * units::distance(50, units::thou) / m_lift, 1 / m_gamma) - 1;
            const double k = std::acos(s) / angle;
            const double extents = constants::pi / k;

            // pi / 2 = k * x

            const double step = extents / (m_steps - 5.0);
            for (int i = 0; i < m_steps; ++i) {
                if (i == 0) {
                    m_function->addSample(0.0, m_lift);
                }
                else {
                    const double x = i * step;
                    const double lift = (x >= extents)
                        ? 0.0
                        : m_lift * std::pow(0.5 + 0.5 * std::cos(k * x), m_gamma);
                    m_function->addSample(x, lift);
                    m_function->addSample(-x, lift);
                }
            }

            m_function->setFilterRadius(step);
        }

    protected:
        double m_durationAt50Thou = 0.0;
        double m_gamma = 1.0;
        double m_lift = 300.0;
        int m_steps = 100;
        double m_cycleAngle = 4 * constants::pi;
        FunctionNode *m_function = nullptr;
    };

    // How far open a port is (0 to 1) around its centre, as a cam lobe profile: ports opened
    // by a rotor flank or a piston crown work like a cam that turns once per cycle. `duration`
    // is from first crack to fully shut and `ramp` how long opening (and closing) takes, both
    // in crank degrees.
    class GeneratePortLobeNode : public Node {
    public:
        GeneratePortLobeNode() { /* void */ }
        virtual ~GeneratePortLobeNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("duration", &m_duration);
            addInput("ramp", &m_ramp);
            addInput("cycle_angle", &m_cycleAngle);
            addInput("steps", &m_steps);
            addInput("function", &m_function, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            const double toCam = 2 * constants::pi / m_cycleAngle;
            const double half = 0.5 * m_duration;
            const double ramp = std::max(1e-6, std::min(m_ramp, half));
            const double extents = half * 1.1;
            const int steps = std::max(m_steps, 8);
            const double step = extents / (steps - 1);

            for (int i = 0; i < steps; ++i) {
                const double x = i * step; // crank angle from the centre
                double open = 0;
                if (x < half - ramp) open = 1;
                else if (x < half) open = 0.5 - 0.5 * std::cos(constants::pi * (half - x) / ramp);

                m_function->addSample(x * toCam, open);
                if (i > 0) m_function->addSample(-x * toCam, open);
            }
            m_function->setFilterRadius(step * toCam);
        }

    protected:
        double m_duration = 0.0;
        double m_ramp = 0.0;
        double m_cycleAngle = 4 * constants::pi;
        int m_steps = 100;
        FunctionNode *m_function = nullptr;
    };

    class AddGearNode : public Node {
    public:
        AddGearNode() { /* void */ }
        virtual ~AddGearNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("ratio", &m_ratio, InputTarget::Type::Object);
            addInput("transmission", &m_transmission, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            m_transmission->addGear(m_ratio);
        }

    protected:
        double m_ratio = 1.0;
        TransmissionNode *m_transmission = nullptr;
    };

    class SetTransmissionNode : public Node {
    public:
        SetTransmissionNode() { /* void */ }
        virtual ~SetTransmissionNode() { /* void */ }

    protected:
        virtual void registerInputs() override {
            addInput("transmission", &m_transmission, InputTarget::Type::Object);

            Node::registerInputs();
        }

        virtual void _evaluate() override {
            readAllInputs();

            Transmission *transmission = new Transmission;
            m_transmission->generate(transmission);
            Compiler::output()->transmission = transmission;
        }

    protected:
        TransmissionNode *m_transmission = nullptr;
    };

    class SetVehicleNode : public piranha::Node {
    public:
        SetVehicleNode() { /* void */ }
        virtual ~SetVehicleNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            registerInput(&m_vehicle, "vehicle");

            Node::registerInputs();
        }

        virtual void _evaluate() {
            VehicleNode *vehicleNode = getObject<VehicleNode>(m_vehicle);

            Vehicle *vehicle = new Vehicle;
            vehicleNode->generate(vehicle);
            Compiler::output()->vehicle = vehicle;
        }

    protected:
        piranha::pNodeInput m_vehicle = nullptr;
    };

    class SetApplicationSettingsNode : public Node {
    public:
        SetApplicationSettingsNode() { /* void */ }
        virtual ~SetApplicationSettingsNode() { /* void */ }

    protected:
        virtual void registerInputs() {
            addInput("start_fullscreen", &m_settings.startFullscreen);
            addInput("power_units", &m_settings.powerUnits);
            addInput("torque_units", &m_settings.torqueUnits);
            addInput("speed_units", &m_settings.speedUnits);
            addInput("pressure_units", &m_settings.pressureUnits);
            addInput("boost_units", &m_settings.boostUnits);

            addInput("color_background", &m_settings.colorBackground);
            addInput("color_foreground", &m_settings.colorForeground);
            addInput("color_shadow", &m_settings.colorShadow);
            addInput("color_highlight1", &m_settings.colorHighlight1);
            addInput("color_highlight2", &m_settings.colorHighlight2);
            addInput("color_pink", &m_settings.colorPink);
            addInput("color_red", &m_settings.colorRed);
            addInput("color_orange", &m_settings.colorOrange);
            addInput("color_yellow", &m_settings.colorYellow);
            addInput("color_blue", &m_settings.colorBlue);
            addInput("color_green", &m_settings.colorGreen);

            Node::registerInputs();
        }

        virtual void _evaluate() {
            readAllInputs();

            Compiler::output()->applicationSettings = m_settings;
        }

    protected:
        ApplicationSettings m_settings;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_ACTIONS_H */
