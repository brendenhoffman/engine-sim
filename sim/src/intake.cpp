#include "intake.h"

#include "units.h"
#include "forced_induction.h"

#include <cmath>

Intake::Intake() {
    m_inputFlowK = 0;
    m_idleFlowK = 0;
    m_flow = 0;
    m_throttle = 1.0;
    m_idleThrottlePlatePosition = 0.0;
    m_crossSectionArea = 0.0;
    m_flowRate = 0;
    m_totalFuelInjected = 0;
    m_molecularAfr = 0;
    m_runnerLength = 0;
}

Intake::~Intake() {
    /* void */
}

void Intake::initialize(Parameters &params) {
    const double width = std::sqrt(params.CrossSectionArea);
    m_system.initialize(
        units::pressure(1.0, units::atm),
        params.volume,
        units::celcius(25.0));
    m_system.setGeometry(
        width,
        params.volume / params.CrossSectionArea,
        1.0,
        0.0);

    m_atmosphere.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1000.0, units::m3),
        units::celcius(25.0));
    m_atmosphere.setGeometry(
        units::distance(100.0, units::m),
        units::distance(100.0, units::m),
        1.0,
        0.0);

    m_inputFlowK = params.InputFlowK;
    m_molecularAfr = params.MolecularAfr;
    m_idleFlowK = params.IdleFlowK;
    m_idleThrottlePlatePosition = params.IdleThrottlePlatePosition;
    m_defaultIdleThrottlePlatePosition = params.IdleThrottlePlatePosition;
    m_runnerLength = params.RunnerLength;
    m_crossSectionArea = params.CrossSectionArea;
    m_velocityDecay = params.VelocityDecay;
    m_runnerFlowRate = params.RunnerFlowRate;
}

void Intake::destroy() {
    delete m_forcedInduction;
    m_forcedInduction = nullptr;
}

void Intake::setGasProperties(const GasSystem::Properties *properties) {
    m_system.setProperties(properties);
    m_atmosphere.setProperties(properties);
    if (m_forcedInduction != nullptr) m_forcedInduction->setGasProperties(properties);
}

void Intake::setForcedInduction(ForcedInduction *forcedInduction) {
    delete m_forcedInduction;
    m_forcedInduction = forcedInduction;
}

GasSystem::Mix Intake::freshMix() const {
    const double ideal_afr = 0.8 * m_molecularAfr * 4;
    const double p_air = ideal_afr / (1 + ideal_afr);
    GasSystem::Mix fuelAirMix;
    fuelAirMix.p_fuel = 1 - p_air;
    fuelAirMix.p_inert = p_air * 0.75;
    fuelAirMix.p_o2 = p_air * 0.25;
    return fuelAirMix;
}

void Intake::process(double dt) {
    const GasSystem::Mix fuelAirMix = freshMix();

    const double idle_afr = 2.0;
    const double p_idle_air = idle_afr / (1 + idle_afr);
    GasSystem::Mix fuelMix;
    fuelMix.p_fuel = (1.0 - p_idle_air);
    fuelMix.p_inert = p_idle_air * 0.75;
    fuelMix.p_o2 = p_idle_air * 0.25;

    const double throttle = getThrottlePlatePosition();
    const double flowAttenuation = std::cos(throttle * constants::pi / 2);

    GasSystem::FlowParameters flowParams;
    flowParams.crossSectionArea_0 = units::area(10, units::m2);
    flowParams.crossSectionArea_1 = m_crossSectionArea;
    flowParams.direction_x = 0.0;
    flowParams.direction_y = -1.0;
    flowParams.dt = dt;

    // The throttle draws from the open air, or from the charge the compressors make
    GasSystem *source = &m_atmosphere;
    if (m_forcedInduction != nullptr) source = m_forcedInduction->charge();
    else m_atmosphere.reset(units::pressure(1.0, units::atm), units::celcius(25.0), fuelAirMix);
    flowParams.system_0 = source;
    flowParams.system_1 = &m_system;
    flowParams.k_flow = flowAttenuation * m_inputFlowK;
    m_flow = m_system.flow(flowParams);

    if (m_forcedInduction == nullptr) m_atmosphere.reset(units::pressure(1.0, units::atm), units::celcius(25.0), fuelMix);
    flowParams.system_0 = source;
    flowParams.system_1 = &m_system;
    flowParams.k_flow = m_idleFlowK;
    const double idleCircuitFlow = m_system.flow(flowParams);

    m_system.dissipateExcessVelocity();
    m_system.updateVelocity(dt, m_velocityDecay);

    if (m_flow > 0) {
        m_totalFuelInjected += fuelAirMix.p_fuel * m_flow;
    }

    if (idleCircuitFlow > 0) {
        m_totalFuelInjected += fuelMix.p_fuel * idleCircuitFlow;
    }
}
