#include "combustion_chamber.h"

#include "constants.h"
#include "units.h"
#include "piston.h"
#include "connecting_rod.h"
#include "utilities.h"
#include "exhaust_system.h"
#include "cylinder_bank.h"
#include "engine.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
    // A flow constant far wider than any runner: an open end
    constexpr double OpenPipeEnd = 1e3;
}

CombustionChamber::CombustionChamber() {
    m_crankcasePressure = 0.0;
    m_piston = nullptr;
    m_head = nullptr;
    m_engine = nullptr;
    m_pistonSpeed = nullptr;
    m_pressure = nullptr;
    m_lit = false;
    m_litLastFrame = false;
    m_peakTemperature = 0;

    m_meanPistonSpeedToTurbulence = nullptr;
    m_nBurntFuel = 0;

    m_manifoldToRunnerFlowRate = 0;
    m_primaryToCollectorFlowRate = 0;
    m_cylinderWidthApproximation = 0;
    m_cylinderCrossSectionSurfaceArea = 0;

    m_lastTimestepTotalExhaustFlow = 0;
    m_lastTimestepTotalIntakeFlow = 0;
    m_exhaustFlow = 0;
    m_exhaustFlowRate = 0;
    m_intakeFlowRate = 0;

    m_fuel = nullptr;
}

CombustionChamber::~CombustionChamber() {
    assert(m_pistonSpeed == nullptr);
    assert(m_pressure == nullptr);
}

void CombustionChamber::initialize(const Parameters &params) {
    m_piston = params.piston;
    m_head = params.Head;
    m_fuel = params.fuel;
    m_crankcasePressure = params.CrankcasePressure;
    m_heatTransferCoefficient = params.HeatTransferCoefficient;
    m_blockTemperature = params.BlockTemperature;
    m_heatTransferReferenceSpeed = params.HeatTransferReferenceSpeed;
    m_random.seed(1 + m_piston->getCylinderIndex() + 97 * m_head->getCylinderBank()->getIndex());
    m_meanPistonSpeedToTurbulence = params.MeanPistonSpeedToTurbulence;

    m_pistonSpeed = new double[StateSamples];
    m_pressure = new double[StateSamples];
    for (int i = 0; i < StateSamples; ++i) {
        m_pistonSpeed[i] = 0;
        m_pressure[i] = 0;
    }

    Intake *intake = m_head->getIntake(m_piston->getCylinderIndex());
    ExhaustSystem *exhaust = m_head->getExhaustSystem(m_piston->getCylinderIndex());

    m_manifoldToRunnerFlowRate = intake->getRunnerFlowRate();
    m_primaryToCollectorFlowRate = exhaust->getPrimaryFlowRate();

    CylinderBank *bank = m_head->getCylinderBank();
    m_rotorFace = bank->isRotor();
    if (m_rotorFace) {
        // The chamber is a long, shallow pocket between two apexes, as wide as the rotor
        m_cylinderCrossSectionSurfaceArea = bank->rotorFlankLength() * bank->getRotorWidth();
    }
    else {
        const double bore_r = bank->getBore() / 2.0;
        m_cylinderCrossSectionSurfaceArea = constants::pi * bore_r * bore_r;
    }
    m_cylinderWidthApproximation = std::sqrt(m_cylinderCrossSectionSurfaceArea);

    const double height = getVolume() / m_cylinderCrossSectionSurfaceArea;
    m_system.setGeometry(
        m_cylinderWidthApproximation,
        height,
        1.0,
        0.0);

    const double intakeRunnerCrossSection = m_head->getIntakeRunnerCrossSectionArea();
    const double intakeRunnerWidth = std::sqrt(intakeRunnerCrossSection);
    const double manifoldRunnerLength = intake->getRunnerLength();
    const double manifoldRunnerVolume = intakeRunnerCrossSection * manifoldRunnerLength;
    const double totalIntakeRunnerVolume = m_head->getIntakeRunnerVolume() + manifoldRunnerVolume;
    const double overallIntakeRunnerLength = totalIntakeRunnerVolume / intakeRunnerCrossSection;
    m_intakeRunnerAndManifold.initialize(
        units::pressure(1.0, units::atm),
        totalIntakeRunnerVolume,
        units::celcius(25.0));
    m_intakeRunnerAndManifold.setGeometry(
        overallIntakeRunnerLength,
        intakeRunnerWidth,
        1.0,
        0.0);

    const double exhaustRunnerCrossSection = m_head->getExhaustRunnerCrossSectionArea();
    const double exhaustRunnerWidth = std::sqrt(exhaustRunnerCrossSection);
    const double exhaustTubeLength =
        exhaust->getPrimaryTubeLength() + m_head->getHeaderPrimaryLength(m_piston->getCylinderIndex());
    const double exhaustTubeVolume = exhaustRunnerCrossSection * exhaustTubeLength;
    const double totalExhaustRunnerVolume = m_head->getExhaustRunnerVolume() + exhaustTubeVolume;
    const double overallExhaustRunnerLength = totalExhaustRunnerVolume / exhaustRunnerCrossSection;
    m_exhaustRunnerAndPrimary.initialize(
        units::pressure(1.0, units::atm),
        totalExhaustRunnerVolume,
        units::celcius(25.0));
    m_exhaustRunnerAndPrimary.setGeometry(
        overallExhaustRunnerLength,
        exhaustRunnerWidth,
        1.0,
        0.0);

    // A rotor's second and third faces share the first face's pipes
    CombustionChamber *owner = nullptr;
    if (params.PipeResolution > 0 && m_rotorFace && m_piston->getCylinderIndex() > 0) {
        for (int j = 0; j < m_engine->getCylinderCount(); ++j) {
            CombustionChamber *other = m_engine->getChamber(j);
            if (other != this && other->m_piston != nullptr && other->m_head == m_head
                && other->m_piston->getCylinderIndex() == 0 && other->hasPipes()) {
                owner = other;
            }
        }
    }
    if (owner != nullptr) {
        m_intakePipeRef = &owner->m_intakePipe;
        m_exhaustPipeRef = &owner->m_exhaustPipe;
        m_intakePortConnection = m_intakePipeRef->connect(1, &m_system);
        m_exhaustPortConnection = m_exhaustPipeRef->connect(0, &m_system);
    }
    else if (params.PipeResolution > 0) {
        Pipe::Parameters pipe;
        pipe.pressure = units::pressure(1.0, units::atm);
        pipe.temperature = units::celcius(25.0);

        // A primed intake at rest: the runner holds fresh mixture, the exhaust plain air
        pipe.mix = intake->freshMix();
        pipe.length = overallIntakeRunnerLength;
        pipe.areas = { intakeRunnerCrossSection };
        pipe.cells = std::max(4, static_cast<int>(std::lround(pipe.length * params.PipeResolution)));
        m_intakePipe.initialize(pipe);

        pipe.mix = GasSystem::Mix();
        pipe.mix.p_o2 = 0.25;
        pipe.mix.p_inert = 0.75;
        pipe.length = overallExhaustRunnerLength;
        pipe.areas = { exhaustRunnerCrossSection };
        pipe.cells = std::max(4, static_cast<int>(std::lround(pipe.length * params.PipeResolution)));
        m_exhaustPipe.initialize(pipe);

        m_intakePipe.connect(0, &intake->m_system);
        m_intakePortConnection = m_intakePipe.connect(1, &m_system);
        m_exhaustPortConnection = m_exhaustPipe.connect(0, &m_system);
        m_exhaustPipe.connect(1, exhaust->getSystem());
        m_intakePipeRef = &m_intakePipe;
        m_exhaustPipeRef = &m_exhaustPipe;
        m_pipeOwner = true;
    }
}

double CombustionChamber::exhaustSoundSignal() const {
    if (hasPipes()) {
        // A shared pipe already carries every face's pulses; it's heard once, through its owner
        if (!m_pipeOwner) return 0;
        const int last = m_exhaustPipe.cellCount() - 1;
        const double u = m_exhaustPipe.cellVelocity(last);
        return (m_exhaustPipe.cellPressure(last) - units::pressure(1.0, units::atm))
            + 0.1 * 0.5 * m_exhaustPipe.cellDensity(last) * u * u;
    }
    return (m_exhaustRunnerAndPrimary.pressure() - units::pressure(1.0, units::atm))
        + 0.1 * m_exhaustRunnerAndPrimary.dynamicPressure(1.0, 0.0)
        + 0.1 * m_exhaustRunnerAndPrimary.dynamicPressure(-1.0, 0.0);
}

double CombustionChamber::exhaustSoundPathLength() const {
    ExhaustSystem *exhaust = m_head->getExhaustSystem(m_piston->getCylinderIndex());
    // With pipes the primary's travel time is simulated; only the rest of the system is left
    return hasPipes()
        ? exhaust->getLength()
        : m_head->getHeaderPrimaryLength(m_piston->getCylinderIndex()) + exhaust->getLength();
}

GasSystem::Mix CombustionChamber::intakeRunnerMix() const {
    return hasPipes() ? m_intakePipeRef->cellMix(m_intakePipeRef->cellCount() - 1) : m_intakeRunnerAndManifold.mix();
}

void CombustionChamber::destroy() {
    if (m_pistonSpeed != nullptr) delete[] m_pistonSpeed;
    if (m_pressure != nullptr) delete[] m_pressure;

    m_pistonSpeed = nullptr;
    m_pressure = nullptr;
}

double CombustionChamber::getVolume() const {
    const double combustionPortVolume = m_head->getCombustionChamberVolume();
    const CylinderBank *bank = m_head->getCylinderBank();

    if (m_rotorFace) {
        const double cycleAngle = m_engine->getOutputCrankshaft()->getCycleAngle();
        return combustionPortVolume + bank->rotorFaceSweptVolume(m_piston->getCylinderIndex(), cycleAngle);
    }

    const double area = bank->boreSurfaceArea();
    const double s =
        m_piston->relativeX() * bank->getDx()
        + m_piston->relativeY() * bank->getDy();
    const double sweep =
        area * (bank->getDeckHeight() - s - m_piston->getCompressionHeight());

    return sweep + combustionPortVolume - m_piston->getDisplacement();
}

double CombustionChamber::rotorVolumeDerivative() const {
    const double cycleAngle = m_engine->getOutputCrankshaft()->getCycleAngle();
    return m_head->getCylinderBank()->rotorFaceVolumeDerivative(m_piston->getCylinderIndex(), cycleAngle);
}

double CombustionChamber::pistonSpeed() const {
    const CylinderBank *bank = m_head->getCylinderBank();

    if (m_rotorFace) {
        // What stirs a rotor chamber is the charge being carried along by the rotor, at about
        // the apex speed relative to the housing (the rotor turns at a third of shaft speed and
        // orbits at e), rather than how fast the volume changes
        const double shaftSpeed = std::abs(m_engine->getOutputCrankshaft()->m_body.v_theta);
        return shaftSpeed * (bank->getRotorGeneratingRadius() / 3 + bank->getRotorEccentricity());
    }

    return
        m_piston->m_body.v_x * bank->getDx()
        + m_piston->m_body.v_y * bank->getDy();
}

double CombustionChamber::calculateMeanPistonSpeed() const {
    double avg = 0;
    for (int i = 0; i < StateSamples; ++i) {
        avg += m_pistonSpeed[i];
    }

    avg /= StateSamples;
    return avg;
}

double CombustionChamber::calculateFiringPressure() const {
    double firingPressure = 0;
    for (int i = 0; i < StateSamples; ++i) {
        if (m_pressure[i] > firingPressure) {
            firingPressure = m_pressure[i];
        }
    }

    return firingPressure;
}

bool CombustionChamber::popLitLastFrame() {
    const bool lit = m_litLastFrame;
    m_litLastFrame = false;

    return lit;
}

void CombustionChamber::ignite() {
    if (!m_lit) {
        if (m_system.mix().p_fuel == 0) return;

        const double afr = m_system.mix().p_o2 / m_system.mix().p_fuel;
        const double equivalenceRatio = afr / m_fuel->getMolecularAfr();
        if (equivalenceRatio < 0.5) return;
        else if (equivalenceRatio > 1.9) return;

        const GasSystem::Mix mix = m_system.mix();
        m_flameEvent.residualFraction = std::clamp(
            (mix.p_inert - 3 * mix.p_o2) / std::max(1e-9, 1 - mix.p_fuel), 0.0, 1.0);
        m_flameEvent.equivalenceRatio = equivalenceRatio;

        // Too much burned gas in the charge for the flame to take: a misfire. The charge
        // goes out unburned, and the next one, cleaner for it, fires harder.
        double residualFactor = m_fuel->residualFlameFactor(m_flameEvent.residualFraction);
        if (m_fuel->getCyclicVariation() > 0) {
            // Stronger turbulence at speed steadies the early flame, so the spread shrinks
            // above idle-like mean piston speeds (~5 m/s)
            const double meanSpeed = std::max(m_pistonSpeedSum / StateSamples, 1e-3);
            const double steadiness = std::min(1.0, 5.0 / meanSpeed);
            const double z = std::normal_distribution<double>(0.0, 1.0)(m_random);
            residualFactor *= std::exp(m_fuel->getCyclicVariation() * m_flameEvent.residualFraction * steadiness * z);
        }
        if (residualFactor < 0.05) {
            ++m_misfires;
            m_lastBurnCompletion = 0;
            return;
        }
        m_lastBurnCompletion = m_flameEvent.percentageLit;

        const double idealInert = m_system.mix().p_o2 / 0.7;
        const double dilution = (m_system.mix().p_inert / idealInert) - 1;

        m_flameEvent.lastVolume = getVolume();
        m_flameEvent.travel_x = 0;
        m_flameEvent.travel_y = 0;
        m_flameEvent.lit_n = 0;
        m_flameEvent.total_n = m_system.n();
        m_flameEvent.percentageLit = 0;
        m_flameEvent.globalMix = m_system.mix();
        m_flameEvent.ignitionAngle = m_engine->getOutputCrankshaft()->getCycleAngle();
        m_lit = true;
        m_litLastFrame = true;

        const double randomness =
            m_fuel->getBurningEfficiencyRandomness();
        const double lowEfficiencyAttenuation =
            m_fuel->getLowEfficiencyAttenuation();
        const double maxBurningEfficiency =
            m_fuel->getMaxBurningEfficiency();
        const double maxTurbulenceEffect =
            m_fuel->getMaxTurbulenceEffect();
        const double maxDilutionEffect =
            m_fuel->getMaxDilutionEffect();

        const double turbulence =
            m_meanPistonSpeedToTurbulence->sampleTriangle(
                calculateMeanPistonSpeed());
        const double mixingFactor =
            1.0 - (
                clamp(turbulence / maxTurbulenceEffect)
                * clamp(1 - dilution / maxDilutionEffect));
        const double rand_s =
            lowEfficiencyAttenuation
            * ((1 - randomness) + randomness * std::uniform_real_distribution<double>(0.0, 1.0)(m_random));
        const double efficiencyAttenuation =
            (mixingFactor * rand_s + (1 - mixingFactor));
        m_flameEvent.efficiency =
            efficiencyAttenuation * maxBurningEfficiency;
        m_flameEvent.flameSpeed = m_fuel->flameSpeed(
            turbulence,
            afr,
            m_system.temperature(),
            m_system.pressure(),
            calculateFiringPressure(),
            units::pressure(160, units::psi));
        m_flameEvent.undilutedFlameSpeed = m_flameEvent.flameSpeed;
        m_flameEvent.flameSpeed *= residualFactor;
    }
}

void CombustionChamber::update(double dt) {
    m_system.setVolume(getVolume());

    updateCycleStates();

    m_intakeFlowRate = m_head->intakeFlowRate(m_piston->getCylinderIndex());
    m_exhaustFlowRate = m_head->exhaustFlowRate(m_piston->getCylinderIndex());

    // Valve/port openings go to the pipes before any of them steps (shared pipes step once,
    // through their owner, with every face's opening in place)
    if (hasPipes()) {
        m_intakePipeRef->setOrifice(1, m_intakeFlowRate, m_intakePortConnection);
        m_exhaustPipeRef->setOrifice(0, m_exhaustFlowRate, m_exhaustPortConnection);
        if (m_pipeOwner) {
            m_intakePipe.setOrifice(0, OpenPipeEnd);
            m_exhaustPipe.setOrifice(1, OpenPipeEnd);
        }
    }
}

void CombustionChamber::flow(double dt) {
    if (m_system.temperature() > m_peakTemperature) {
        m_peakTemperature = m_system.temperature();
    }

    const double volume = getVolume();
    const double cylinderHeight = volume / m_cylinderCrossSectionSurfaceArea;
    const CylinderBank *chamberBank = m_head->getCylinderBank();
    const double cylinderSurfaceArea = m_rotorFace
        ? m_cylinderCrossSectionSurfaceArea * 2
            + 2 * (chamberBank->rotorFlankLength() + chamberBank->getRotorWidth()) * cylinderHeight
        : cylinderHeight * constants::pi * chamberBank->getBore()
            + m_cylinderCrossSectionSurfaceArea * 2;

    const double dT = m_blockTemperature - m_system.temperature();

    double heatTransfer = m_heatTransferCoefficient;
    if (m_heatTransferReferenceSpeed > 0) {
        const double meanSpeed = std::max(m_pistonSpeedSum / StateSamples, 0.05 * m_heatTransferReferenceSpeed);
        heatTransfer *= std::pow(meanSpeed / m_heatTransferReferenceSpeed, 0.8);
    }
    m_system.changeEnergy(dT * cylinderSurfaceArea * heatTransfer * dt);
    m_system.flow(m_piston->getBlowbyK(), dt, m_crankcasePressure, units::celcius(25.0));

    Intake *intake = m_head->getIntake(m_piston->getCylinderIndex());
    ExhaustSystem *exhaust = m_head->getExhaustSystem(m_piston->getCylinderIndex());

    const double start_n = m_system.n();

    double intakeFlow = 0, exhaustFlow = 0;
    if (hasPipes()) {
        // Plenum -> intake pipe -> chamber -> exhaust pipe -> collector; waves in the pipes
        // The runner meets the plenum, and the primary the collector, as open pipe ends (set in
        // update()): their lumped-model flow constants stood in for the whole runner's
        // restriction, which the pipe now provides itself. The valves are the orifices.
        if (m_pipeOwner) {
            m_intakePipe.step(dt);
            m_exhaustPipe.step(dt);
            m_intakePipe.takeFlow(0);
            m_exhaustPipe.takeFlow(1);
        }
        intakeFlow = -m_intakePipeRef->takeFlow(1, m_intakePortConnection);
        exhaustFlow = m_exhaustPipeRef->takeFlow(0, m_exhaustPortConnection);
    }
    else {
        GasSystem::FlowParameters flowParams;
        flowParams.dt = dt;

        flowParams.k_flow = m_manifoldToRunnerFlowRate;
        flowParams.crossSectionArea_0 = intake->getPlenumCrossSectionArea();
        flowParams.crossSectionArea_1 = m_head->getIntakeRunnerCrossSectionArea();
        flowParams.direction_x = 1.0;
        flowParams.direction_y = 0.0;
        flowParams.system_0 = &intake->m_system;
        flowParams.system_1 = &m_intakeRunnerAndManifold;
        GasSystem::flow(flowParams);

        m_intakeRunnerAndManifold.dissipateExcessVelocity();

        flowParams.k_flow = m_intakeFlowRate;
        flowParams.crossSectionArea_0 = m_head->getIntakeRunnerCrossSectionArea();
        flowParams.crossSectionArea_1 = volume / cylinderHeight;
        flowParams.direction_x = 1.0;
        flowParams.direction_y = 0.0;
        flowParams.system_0 = &m_intakeRunnerAndManifold;
        flowParams.system_1 = &m_system;
        intakeFlow = GasSystem::flow(flowParams);

        m_intakeRunnerAndManifold.dissipateExcessVelocity();
        m_system.dissipateExcessVelocity();

        flowParams.k_flow = m_exhaustFlowRate;
        flowParams.crossSectionArea_0 = volume / cylinderHeight;
        flowParams.crossSectionArea_1 = m_head->getExhaustRunnerCrossSectionArea();
        flowParams.direction_x = 1.0;
        flowParams.direction_y = 0.0;
        flowParams.system_0 = &m_system;
        flowParams.system_1 = &m_exhaustRunnerAndPrimary;
        exhaustFlow = GasSystem::flow(flowParams);

        m_system.dissipateExcessVelocity();
        m_exhaustRunnerAndPrimary.dissipateExcessVelocity();

        flowParams.k_flow = m_primaryToCollectorFlowRate;
        flowParams.crossSectionArea_0 = m_head->getExhaustRunnerCrossSectionArea();
        flowParams.crossSectionArea_1 = exhaust->getCollectorCrossSectionArea();
        flowParams.direction_x = 1.0;
        flowParams.direction_y = 0.0;
        flowParams.system_0 = &m_exhaustRunnerAndPrimary;
        flowParams.system_1 = exhaust->getSystem();
        GasSystem::flow(flowParams);

        m_intakeRunnerAndManifold.updateVelocity(dt, intake->getVelocityDecay());
        m_system.updateVelocity(dt, 0.5);
        m_exhaustRunnerAndPrimary.updateVelocity(dt, exhaust->getVelocityDecay());
    }

    if (std::abs(intakeFlow) > 1E-9 && m_lit) {
        m_lit = false;
    }

    m_exhaustFlow = exhaustFlow;
    m_lastTimestepTotalExhaustFlow += exhaustFlow;
    m_lastTimestepTotalIntakeFlow += intakeFlow;

    if (m_lit && m_rotorFace) {
        // A flame runs along the chamber from each plug, both ways, until together they've
        // covered the flank. The leading plug's flame (travel_x) starts at ignition; the
        // trailing plugs' (travel_y) once the shaft has turned the trailing delay further.
        CylinderBank *bank = m_head->getCylinderBank();
        const double flank = bank->rotorFlankLength();
        const int trailingPlugs = bank->getSparkPlugs() - 1;
        const double burnedBefore = std::fmin(1.0,
            (2 * m_flameEvent.travel_x + 2 * trailingPlugs * m_flameEvent.travel_y) / flank);

        Crankshaft *shaft = m_engine->getOutputCrankshaft();
        const double sinceIgnition = std::fmod(
            shaft->getCycleAngle() - m_flameEvent.ignitionAngle + 2 * shaft->getCycleLength(), shaft->getCycleLength());
        m_flameEvent.travel_x += dt * m_flameEvent.flameSpeed;
        if (trailingPlugs > 0 && sinceIgnition >= bank->getTrailingDelay()) {
            m_flameEvent.travel_y += dt * m_flameEvent.flameSpeed;
        }
        const double burnedNow = std::fmin(1.0,
            (2 * m_flameEvent.travel_x + 2 * trailingPlugs * m_flameEvent.travel_y) / flank);

        if (burnedBefore < burnedNow) {
            const double fraction = burnedNow - burnedBefore;
            const double n = fraction * m_system.n();
            const double fuelBurned =
                m_system.react(n * m_flameEvent.efficiency, m_flameEvent.globalMix);
            const double massFuelBurned = fuelBurned * m_fuel->getMolecularMass();
            m_system.changeEnergy(massFuelBurned * m_fuel->getEnergyDensity());

            m_flameEvent.lit_n += n;
            m_flameEvent.percentageLit += fraction;
            m_nBurntFuel += massFuelBurned;
        }
        else {
            m_lit = false;
        }

        m_flameEvent.lastVolume = volume;
    }
    else if (m_lit) {
        CylinderBank *bank = m_head->getCylinderBank();
        const double totalTravel_x = bank->getBore() / 2;
        const double totalTravel_y = volume / bank->boreSurfaceArea();
        const double expansion = volume / m_flameEvent.lastVolume;
        const double lastTravel_x = m_flameEvent.travel_x;
        const double lastTravel_y = m_flameEvent.travel_y * expansion;
        const double flameSpeed = m_flameEvent.flameSpeed;

        m_flameEvent.travel_x =
            std::fmin(lastTravel_x + dt * flameSpeed, totalTravel_x);
        m_flameEvent.travel_y =
            std::fmin(lastTravel_y + dt * flameSpeed, totalTravel_y);

        if (lastTravel_x < m_flameEvent.travel_x || lastTravel_y < m_flameEvent.travel_y) {
            const double burnedVolume =
                m_flameEvent.travel_x * m_flameEvent.travel_x
                * constants::pi * m_flameEvent.travel_y;
            const double prevBurnedVolume =
                lastTravel_x * lastTravel_x * constants::pi * lastTravel_y;
            const double litVolume = burnedVolume - prevBurnedVolume;
            const double n = (litVolume / volume) * m_system.n();

            const double fuelBurned =
                m_system.react(n * m_flameEvent.efficiency, m_flameEvent.globalMix);
            const double massFuelBurned = fuelBurned * m_fuel->getMolecularMass();
            m_system.changeEnergy(
                massFuelBurned * m_fuel->getEnergyDensity());

            m_flameEvent.lit_n += n;
            m_flameEvent.percentageLit += litVolume / volume;

            m_nBurntFuel += massFuelBurned;
        }
        else {
            m_lit = false;
        }

        m_flameEvent.lastVolume = volume;
    }
}

double CombustionChamber::lastEventAfr() const {
    const double totalFuel = m_flameEvent.globalMix.p_fuel * m_flameEvent.total_n;
    const double totalOxygen = m_flameEvent.globalMix.p_o2 * m_flameEvent.total_n;
    const double totalInert = m_flameEvent.globalMix.p_inert * m_flameEvent.total_n;

    constexpr double octaneMolarMass = units::mass(114.23, units::g);
    constexpr double oxygenMolarMass = units::mass(31.9988, units::g);
    constexpr double nitrogenMolarMass = units::mass(28.014, units::g);

    if (totalFuel == 0) return 0;
    else {
        return
            (oxygenMolarMass * totalOxygen + totalInert * nitrogenMolarMass)
            / (totalFuel * octaneMolarMass);
    }
}

double CombustionChamber::calculateFrictionForce(double v_s) const {
    const double cylinderWallForce = m_piston->calculateCylinderWallForce();

    const double F_coul = m_frictionModel.frictionCoeff * cylinderWallForce;
    const double v_st = m_frictionModel.breakawayFrictionVelocity * constants::root_2;
    const double v_coul = m_frictionModel.breakawayFrictionVelocity / 10;
    const double F_brk = m_frictionModel.breakawayFriction;
    const double v = std::abs(v_s);

    const double F_0 = constants::root_2 * constants::e * (F_brk - F_coul);
    const double F_1 = v / v_st;
    const double F_2 = std::exp(-F_1 * F_1) * F_1;
    const double F_3 = F_coul * std::tanh(v / v_coul);
    const double F_4 = m_frictionModel.viscousFrictionCoefficient * v;

    return F_0 * F_2 + F_3 + F_4;
}

void CombustionChamber::updateCycleStates() {
    double crankAngle = m_engine->getOutputCrankshaft()->getCycleAngle();
    if (std::isnan(crankAngle) || std::isinf(crankAngle)) {
        crankAngle = 0.0;
    }

    const double cycle = m_engine->getOutputCrankshaft()->getCycleLength();
    const int i = std::clamp((int)std::round((crankAngle / cycle) * (StateSamples - 1.0)), 0, StateSamples - 1);

    const double speed = std::abs(pistonSpeed());
    m_pistonSpeedSum += speed - m_pistonSpeed[i];
    m_pistonSpeed[i] = speed;
    m_pressure[i] = m_system.pressure();
}

void CombustionChamber::apply(atg_scs::SystemState *system) {
    CylinderBank *bank = m_head->getCylinderBank();

    if (m_rotorFace) {
        // Virtual work: the gas does P dV of work as the shaft turns, so the torque in the
        // engine's direction of rotation is P dV/dphi. The cycle angle runs against the
        // body's theta, hence the sign. (A uniform pressure on all three faces cancels out.)
        const double pressureDifferential = m_system.pressure() - m_crankcasePressure;
        const int shaft = m_engine->getOutputCrankshaft()->m_body.index;
        system->t[shaft] += -pressureDifferential * rotorVolumeDerivative();
        return;
    }
    const double area = (bank->getBore() * bank->getBore() / 4.0) * constants::pi;
    const double v_x = system->v_x[m_piston->m_body.index];
    const double v_y = system->v_y[m_piston->m_body.index];

    const double v_s =
        v_x * bank->getDx() + v_y * bank->getDy();

    const double pressureDifferential = m_system.pressure() - m_crankcasePressure;
    const double force = -area * pressureDifferential;

    if (std::isnan(force) || std::isinf(force)) {
        assert(false);
    }

    constexpr double limit = 1E-3;
    const double abs_v_s = std::fmin(std::abs(v_s), limit);
    const double attenuation = abs_v_s / limit;

    const double F = calculateFrictionForce(v_s) * attenuation;
    const double F_fric = (v_s > 0)
        ? -F
        : F;

    system->applyForce(
        0.0,
        0.0,
        (force + F_fric) * bank->getDx(),
        (force + F_fric) * bank->getDy(),
        m_piston->m_body.index);
}

double CombustionChamber::getFrictionForce() const {
    if (m_rotorFace) return 0; // Apex and side seal friction is part of the shaft's friction torque
    CylinderBank *bank = m_head->getCylinderBank();
    const double v_x = m_piston->m_body.v_x;
    const double v_y = m_piston->m_body.v_y;

    const double v_s =
        v_x * bank->getDx() + v_y * bank->getDy();

    return calculateFrictionForce(v_s);
}
