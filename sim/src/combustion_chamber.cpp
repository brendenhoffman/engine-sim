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

    // A crankcase-scavenged two-stroke's crankcase, primed with fresh charge
    if (hasCrankcase()) {
        const double area = constants::pi * m_head->getCylinderBank()->getBore() * m_head->getCylinderBank()->getBore() / 4;
        const double swept = area * 2 * m_piston->getRod()->getCrankshaft()->getThrow();
        const double ratio = std::max(m_engine->getTwoStroke().crankcaseRatio, 1.05);
        m_crankcase.initialize(units::pressure(1.0, units::atm), swept / (ratio - 1) + swept, units::celcius(25.0), intake->freshMix());
        m_crankcase.setGeometry(std::sqrt(area), std::sqrt(area), 1.0, 0.0);
    }

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
        const Engine::Parameters::TwoStroke &ts = m_engine->getTwoStroke();
        if (ts.bellyDiameter > 0) {
            // A two-stroke's expansion chamber: header, diffuser cone, belly, baffle cone, stinger
            const double sections[5] = { ts.headerLength, ts.diffuserLength, ts.bellyLength, ts.baffleLength, ts.stingerLength };
            const double ends[5][2] = {
                { ts.headerDiameter, ts.headerDiameter }, { ts.headerDiameter, ts.bellyDiameter },
                { ts.bellyDiameter, ts.bellyDiameter }, { ts.bellyDiameter, ts.stingerDiameter },
                { ts.stingerDiameter, ts.stingerDiameter } };
            pipe.length = sections[0] + sections[1] + sections[2] + sections[3] + sections[4];
            pipe.cells = std::max(8, static_cast<int>(std::lround(pipe.length * params.PipeResolution)));
            pipe.areas.resize(pipe.cells);
            for (int c = 0; c < pipe.cells; ++c) {
                double x = (c + 0.5) * pipe.length / pipe.cells;
                double d = ts.stingerDiameter;
                for (int k = 0; k < 5; ++k) {
                    if (x <= sections[k] || k == 4) {
                        const double f = (sections[k] > 0) ? std::clamp(x / sections[k], 0.0, 1.0) : 1.0;
                        d = ends[k][0] + (ends[k][1] - ends[k][0]) * f;
                        break;
                    }
                    x -= sections[k];
                }
                pipe.areas[c] = constants::pi * d * d / 4;
            }
        }
        m_exhaustPipe.initialize(pipe);

        m_intakePipe.connect(0, &intake->m_system);
        // A crankcase-scavenged two-stroke breathes into its crankcase
        m_intakePortConnection = m_intakePipe.connect(1, hasCrankcase() ? &m_crankcase : &m_system);
        m_exhaustPortConnection = m_exhaustPipe.connect(0, &m_system);
        m_exhaustPipe.connect(1, exhaust->getSystem());
        m_intakePipeRef = &m_intakePipe;
        m_exhaustPipeRef = &m_exhaustPipe;
        m_pipeOwner = true;
    }
}

// While the Jake brake works its releases are heard smoothed: the charge bleeds out through a
// cracked-open valve, a chug rather than the sharp ring a full blowdown gives
double CombustionChamber::exhaustSoundSignal() const {
    const double raw = rawExhaustSoundSignal();
    return raw + m_jakeBlend * (m_jakeSound - raw);
}

double CombustionChamber::rawExhaustSoundSignal() const {
    if (hasPipes()) {
        // A shared pipe already carries every face's pulses; it's heard once, through its owner
        if (!m_pipeOwner) return 0;
        const int last = m_exhaustPipe.cellCount() - 1;
        const double u = m_exhaustPipe.cellVelocity(last);
        return (m_exhaustPipe.cellPressure(last) - units::pressure(1.0, units::atm))
            + 0.1 * 0.5 * m_exhaustPipe.cellDensity(last) * u * u;
    }
    // Against the manifold rather than the open air, when the exhaust system asks: behind a
    // turbo the manifold sits well above ambient and smears each cylinder's blowdown into a
    // plateau; what gets through the turbine is the pulses on top of it
    ExhaustSystem *exhaust = m_head->getExhaustSystem(m_piston->getCylinderIndex());
    const double ambient = units::pressure(1.0, units::atm);
    // By load: a diesel's fueling share, or the pedal
    const double load = m_fuel->isCompressionIgnition() ? m_engine->getDieselFueling() : m_engine->getSpeedControl();
    const double reference = ambient + exhaust->manifoldRelativeSoundAt(std::clamp(load, 0.0, 1.0)) * (exhaust->getSmoothedPressure() - ambient);
    return (m_exhaustRunnerAndPrimary.pressure() - reference)
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
    {
        static const bool trace = std::getenv("ENGINE_SIM_2T_TRACE") != nullptr;
        if (trace && isTwoStroke()) {
            std::fprintf(stderr, "2t ignite: cyl %.0f kPa %.0f K  trapped %.3f mmol, fuel %.2f mg, o2 %.2f mmol, burned-gas share %.2f  lit %d\n",
                m_system.pressure() / 1000, m_system.temperature(), m_system.n() * 1000, m_system.n_fuel() * m_fuel->getMolecularMass() * 1e6,
                m_system.n_o2() * 1000, std::clamp((m_system.mix().p_inert - 3 * m_system.mix().p_o2) / std::max(1e-9, 1 - m_system.mix().p_fuel), 0.0, 1.0), m_lit);
        }
    }
    if (m_fuel->isCompressionIgnition()) {
        injectDiesel();
        return;
    }

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
        if (m_fuel->getMisfireResidual() > 0) {
            const double z = std::normal_distribution<double>(0.0, 1.0)(m_random);
            if (m_flameEvent.residualFraction * std::exp(m_fuel->getMisfireSpread() * z) > m_fuel->getMisfireResidual()) {
                ++m_misfires;
                m_lastBurnCompletion = 0;
                return;
            }
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
    if (isTwoStroke()) updateTwoStrokePorts(dt);

    // Jake brake: off throttle, crack the exhaust valve open at the top of compression and hold it
    // through the expansion stroke, so the compressed air bleeds out (a chug, through the small
    // opening) instead of pushing the piston back down
    const double volume = getVolume();
    m_volumeMin = std::min(m_volumeMin, volume);
    m_volumeMax = std::max(m_volumeMax, volume);
    if (m_engine->isJakeBrakeActive() && !m_rotorFace) {
        const double swept = m_volumeMax - m_volumeMin;
        // At the top of compression: the piston has done its work squeezing the charge, and
        // letting it go now means none comes back on the way down
        if (!m_jakeOpen && volume >= m_lastVolume && volume < m_volumeMin + 0.05 * swept
            && m_system.pressure() > units::pressure(5.0, units::atm)) {
            m_jakeOpen = true;
        }
        else if (m_jakeOpen && volume > m_volumeMin + 0.95 * swept) {
            m_jakeOpen = false;
        }
        if (m_jakeOpen) m_exhaustFlowRate = std::max(m_exhaustFlowRate, m_engine->getJakeBrakeFlow());
    }
    else {
        m_jakeOpen = false;
    }
    m_lastVolume = volume;
    if (m_engine->getJakeBrakeFlow() > 0) {
        m_jakeSound += std::min(1.0, dt * 2 * constants::pi * 100.0) * (rawExhaustSoundSignal() - m_jakeSound);
        m_jakeBlend += std::clamp((m_engine->isJakeBrakeActive() ? 1.0 : 0.0) - m_jakeBlend, -dt / 0.1, dt / 0.1);
    }

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
    double chargeFlow = 0;   // Into the cylinder (on a crankcase two-stroke, through the transfers)
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
        if (hasCrankcase()) transferFlow(dt, &chargeFlow);
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
        flowParams.system_1 = hasCrankcase() ? &m_crankcase : &m_system;
        intakeFlow = GasSystem::flow(flowParams);
        if (hasCrankcase()) transferFlow(dt, &chargeFlow);

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

    if (!hasCrankcase()) chargeFlow = intakeFlow;
    {
        static const bool trace = std::getenv("ENGINE_SIM_2T_TRACE") != nullptr;
        static int count = 0;
        if (trace && isTwoStroke() && (++count % 40) == 0) {
            const double a = angleFromTdc(pistonTravel(), pistonTravel() >= m_lastPistonTravel - 1e-9) * 180 / constants::pi;
            std::fprintf(stderr, "2t %6.1f deg  cyl %6.1f kPa  cc %6.1f kPa  inlet %.4f transfer %.4f exhaust %.4f  in %+.2e tr %+.2e ex %+.2e  fuel %.3f\n",
                a, m_system.pressure() / 1000, (hasCrankcase() ? m_crankcase : m_head->getIntake(m_piston->getCylinderIndex())->m_system).pressure() / 1000, m_intakeFlowRate, m_transferFlowRate, m_exhaustFlowRate,
                intakeFlow, chargeFlow, exhaustFlow, m_system.mix().p_fuel);
        }
    }
    if (std::abs(chargeFlow) > 1E-9 && m_lit) {
        m_lit = false;
    }
    if (std::abs(chargeFlow) > 1E-9 && m_diesel.active) {
        m_diesel.active = false; // Whatever's left goes out with the exhaust
    }
    if (m_diesel.active) burnDiesel(dt);

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

bool CombustionChamber::isTwoStroke() const {
    return m_engine != nullptr && m_engine->getTwoStroke().enabled() && !m_rotorFace;
}

bool CombustionChamber::hasCrankcase() const {
    return isTwoStroke() && m_engine->getTwoStroke().crankcaseRatio > 0;
}

// Piston travel down from TDC at a crank angle after TDC (slider-crank)
double CombustionChamber::travelAt(double a) const {
    const double r = m_piston->getRod()->getCrankshaft()->getThrow();
    const double l = m_piston->getRod()->getLength();
    const double s = r * std::sin(a);
    return r * (1 - std::cos(a)) + l - std::sqrt(std::max(l * l - s * s, 0.0));
}

double CombustionChamber::pistonTravel() const {
    const CylinderBank *bank = m_head->getCylinderBank();
    const double area = constants::pi * bank->getBore() * bank->getBore() / 4;
    const double stroke = 2 * m_piston->getRod()->getCrankshaft()->getThrow();
    return std::clamp((getVolume() - m_head->getCombustionChamberVolume()) / area, 0.0, stroke);
}

// Crank angle after TDC from the piston's travel and which way it's going
double CombustionChamber::angleFromTdc(double travel, bool falling) const {
    double lo = 0, hi = constants::pi;
    for (int i = 0; i < 30; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (travelAt(mid) < travel) lo = mid; else hi = mid;
    }
    const double a = 0.5 * (lo + hi);
    return falling ? a : 2 * constants::pi - a;
}

void CombustionChamber::updateTwoStrokePorts(double dt) {
    const Engine::Parameters::TwoStroke &ts = m_engine->getTwoStroke();
    const double stroke = 2 * m_piston->getRod()->getCrankshaft()->getThrow();
    const double travel = pistonTravel();
    const bool falling = travel >= m_lastPistonTravel;
    m_lastPistonTravel = travel;

    // A port centred on BDC, open for `duration`: its top sits where the crown is at half that
    // before BDC, and its open area grows with how far below that the crown has gone
    auto bdcPort = [&](double duration) {
        const double top = travelAt(constants::pi - duration / 2);
        return (travel > top) ? (travel - top) / std::max(stroke - top, 1e-6) : 0.0;
    };

    if (ts.exhaustPortDuration > 0) {
        double duration = ts.exhaustPortDuration;
        if (ts.exhaustPortDurationLow > 0 && ts.powerValveEnd > ts.powerValveStart) {
            const double target = std::clamp((std::abs(m_engine->getSpeed()) - ts.powerValveStart)
                / (ts.powerValveEnd - ts.powerValveStart), 0.0, 1.0);
            m_powerValve += (target - m_powerValve) * std::min(1.0, dt / 0.15);
            duration = ts.exhaustPortDurationLow + (ts.exhaustPortDuration - ts.exhaustPortDurationLow) * m_powerValve;
        }
        m_exhaustFlowRate = ts.exhaustPortFlow * bdcPort(duration);
    }
    m_transferFlowRate = ts.transferPortFlow * bdcPort(ts.transferPortDuration);

    // Scavenging: while the transfers flow and the exhaust is open, the fresh charge pushes
    // the burned gas out ahead of it, so what leaves is mostly burned gas until the cylinder is
    // mostly fresh: fresh share f^(1 + quality) of the gas going out, rather than f
    if (m_transferFlowRate > 0 && m_exhaustFlowRate > 0 && ts.scavengingQuality > 0) {
        const GasSystem::Mix mix = m_system.mix();
        const double burned = std::clamp((mix.p_inert - 3 * mix.p_o2) / std::max(1e-9, 1 - mix.p_fuel), 0.0, 1.0);
        const double fresh = 1 - burned;
        const double freshOut = std::pow(fresh, 1 + ts.scavengingQuality);
        // Fresh charge is the transfers' (the crankcase's, or the intake's) mix; the burned gas
        // whatever makes up the rest of the cylinder's
        const GasSystem::Mix f = hasCrankcase() ? m_crankcase.mix() : m_head->getIntake(m_piston->getCylinderIndex())->freshMix();
        GasSystem::Mix b;
        if (burned > 1e-6) {
            b.p_fuel = std::max(0.0, (mix.p_fuel - fresh * f.p_fuel) / burned);
            b.p_o2 = std::max(0.0, (mix.p_o2 - fresh * f.p_o2) / burned);
            b.p_inert = std::max(0.0, (mix.p_inert - fresh * f.p_inert) / burned);
        }
        else {
            b = mix;
        }
        GasSystem::Mix out;
        out.p_fuel = freshOut * f.p_fuel + (1 - freshOut) * b.p_fuel;
        out.p_o2 = freshOut * f.p_o2 + (1 - freshOut) * b.p_o2;
        out.p_inert = freshOut * f.p_inert + (1 - freshOut) * b.p_inert;
        const double total = out.p_fuel + out.p_o2 + out.p_inert;
        if (total > 0) { out.p_fuel /= total; out.p_o2 /= total; out.p_inert /= total; }
        m_system.setOutflowMix(out);
    }
    else {
        m_system.clearOutflowMix();
    }

    if (!hasCrankcase()) {
        // A blower-fed airbox: the transfers (intake ports) draw straight from the intake
        m_intakeFlowRate = m_transferFlowRate;
        return;
    }

    // The piston rising leaves room under it
    const CylinderBank *bank = m_head->getCylinderBank();
    const double area = constants::pi * bank->getBore() * bank->getBore() / 4;
    const double swept = area * stroke;
    const double ratio = std::max(ts.crankcaseRatio, 1.05);
    m_crankcase.setVolume(swept / (ratio - 1) + swept - area * travel);

    double open = 0;
    if (ts.inletType == 1) {
        // Reed: pushed open by the pressure across it, a little behind
        const double upstream = (m_intakePipeRef != nullptr)
            ? m_intakePipeRef->cellPressure(m_intakePipeRef->cellCount() - 1)
            : m_intakeRunnerAndManifold.pressure();
        const double target = std::clamp((upstream - m_crankcase.pressure()) / ts.reedPressure, 0.0, 1.0);
        m_reed += (target - m_reed) * std::min(1.0, dt / ts.reedResponse);
        open = m_reed;
    }
    else if (ts.inletType == 2) {
        // Rotary disc: open between two crank angles, with short ramps as its edge sweeps the port
        const double a = angleFromTdc(travel, falling);
        const double span = std::fmod(ts.discClose - ts.discOpen + 4 * constants::pi, 2 * constants::pi);
        const double into = std::fmod(a - ts.discOpen + 4 * constants::pi, 2 * constants::pi);
        const double ramp = 15 * constants::pi / 180;
        if (into < span) open = std::clamp(std::min(into, span - into) / ramp, 0.0, 1.0);
    }
    else {
        // Piston port: the skirt uncovers it around TDC
        const double bottom = travelAt(ts.inletDuration / 2);
        open = (travel < bottom) ? (bottom - travel) / std::max(bottom, 1e-6) : 0.0;
    }
    m_intakeFlowRate = ts.inletFlow * open;
}

// Crankcase to cylinder through the transfers
void CombustionChamber::transferFlow(double dt, double *transfer) {
    *transfer = 0;
    if (m_transferFlowRate <= 0) return;
    GasSystem::FlowParameters flowParams;
    flowParams.dt = dt;
    flowParams.k_flow = m_transferFlowRate;
    flowParams.crossSectionArea_0 = 0;
    flowParams.crossSectionArea_1 = 0;
    flowParams.direction_x = 0.0;
    flowParams.direction_y = 1.0;
    flowParams.system_0 = &m_crankcase;
    flowParams.system_1 = &m_system;
    *transfer = GasSystem::flow(flowParams);
}

void CombustionChamber::injectDiesel() {
    const Engine::Parameters::Diesel &pump = m_engine->getDiesel();
    const double share = m_engine->getDieselFueling();
    const double o2PerFuel = m_fuel->getMolecularAfr();
    const double smokeLimit = pump.maxEquivalence * m_system.n_o2() / o2PerFuel;
    const double fuel = (pump.maxFuelMass > 0)
        ? std::min(share * pump.maxFuelMass / m_fuel->getMolecularMass(), smokeLimit)
        : share * smokeLimit;
    // Uneven injectors and shot-to-shot variation
    std::normal_distribution<double> normal(0.0, 1.0);
    if (m_injectorBias <= 0) m_injectorBias = std::max(0.5, 1 + m_fuel->getInjectorSpread() * normal(m_random));
    const double shot = std::max(0.3, 1 + m_fuel->shotVariationAt(share) * normal(m_random));
    m_delayScale = std::exp(m_fuel->getDelayVariation() * normal(m_random));
    const double delivered = std::min(fuel * m_injectorBias * shot, smokeLimit * 1.1);
    const double phi = (m_system.n_o2() > 0) ? delivered * o2PerFuel / m_system.n_o2() : 0.0;
    if (fuel <= 0) {
        m_diesel.active = false;
        return;
    }

    // The fuel goes in as a liquid near 330 K and takes its heat of vaporization from the air
    GasSystem::Mix pure;
    pure.p_fuel = 1;
    pure.p_inert = 0;
    pure.p_o2 = 0;
    constexpr double FuelT = 330.0;
    constexpr double Vaporization = 270e3; // J/kg
    m_system.gainN(delivered, m_system.energyPerMol(FuelT, pure), pure);
    m_system.changeEnergy(-delivered * m_fuel->getMolecularMass() * Vaporization);

    m_diesel = DieselEvent();
    m_diesel.active = true;
    m_diesel.injected = delivered;
    m_diesel.equivalence = phi;
}

void CombustionChamber::burnDiesel(double dt) {
    Crankshaft *shaft = m_engine->getOutputCrankshaft();
    const double rpm = std::max(std::abs(m_engine->getRpm()), 100.0);

    if (!m_diesel.burning) {
        // Ignition delay (Hardenberg & Hase), in crank degrees, from the charge as it is now;
        // integrated so a charge that's still heating up lights when it's ready
        m_diesel.elapsed += dt;
        const double p = m_system.pressure() / 1e5;   // bar
        const double T = m_system.temperature();
        if (p > 12.5 && T > 300) {
            const double activation = 618840.0 / (m_fuel->getCetaneNumber() + 25);
            const double meanSpeed = calculateMeanPistonSpeed();
            const double degrees = (0.36 + 0.22 * meanSpeed)
                * std::exp(activation * (1 / (8.3143 * T) - 1 / 17190.0) + std::pow(21.2 / (p - 12.4), 0.63));
            const double seconds = m_delayScale * degrees / (6 * rpm);
            m_diesel.delay += dt / seconds;
        }
        if (m_diesel.delay >= 1) {
            m_diesel.burning = true;
            m_diesel.startAngle = shaft->getCycleAngle();
            // Watson: the longer the delay, the more fuel has mixed and burns at once
            const double delayMs = std::max(m_diesel.elapsed * 1000, 0.05);
            m_diesel.premixed = std::clamp(
                1 - 0.926 * std::pow(std::max(m_diesel.equivalence, 0.01), 0.37) / std::pow(delayMs, 0.26), 0.05, 0.9);
            m_litLastFrame = true;
            static const bool trace = std::getenv("ENGINE_SIM_DIESEL_TRACE") != nullptr;
            if (trace) {
                std::fprintf(stderr, "diesel lit: bank %d cyl %d  %.0f rpm  delay %.2f ms  premixed %.2f  phi %.2f  p %.0f bar  T %.0f K  fuel %.1f mg\n",
                    m_head->getCylinderBank()->getIndex(), m_piston->getCylinderIndex(),
                    rpm, m_diesel.elapsed * 1000, m_diesel.premixed, m_diesel.equivalence, m_system.pressure() / 1e5, m_system.temperature(),
                    m_diesel.injected * m_fuel->getMolecularMass() * 1e6);
            }
        }
        return;
    }

    // Burned so far: the premixed spike (a few degrees) and the diffusion burn (as fast as the
    // fuel mixes: longer at speed and with more fuel), each a Wiebe curve
    const double sinceStart = std::fmod(shaft->getCycleAngle() - m_diesel.startAngle + 2 * shaft->getCycleLength(),
        shaft->getCycleLength()) * 180 / constants::pi;
    const double premixedDuration = 6 + 4 * rpm / 2000;
    const double diffusionDuration = 30 + 20 * rpm / 2000 + 20 * m_diesel.equivalence;
    auto wiebe = [](double x, double m) { return 1 - std::exp(-6.9 * std::pow(std::clamp(x, 0.0, 2.0), m + 1)); };
    const double burnedShare = m_diesel.premixed * wiebe(sinceStart / premixedDuration, 2.0)
        + (1 - m_diesel.premixed) * wiebe(sinceStart / diffusionDuration, 0.5);
    const double target = burnedShare * m_diesel.injected * m_fuel->getMaxBurningEfficiency();
    const double step = target - m_diesel.burned;
    if (step > 0) {
        const double burned = m_system.burnFuel(step, m_fuel->getMolecularAfr());
        const double mass = burned * m_fuel->getMolecularMass();
        m_system.changeEnergy(mass * m_fuel->getEnergyDensity());
        m_nBurntFuel += mass;
        m_diesel.burned = target;
    }
    if (burnedShare > 0.999) m_diesel.active = false;
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
        const double pressureDifferential = m_system.pressure() - crankcasePressure();
        const int shaft = m_engine->getOutputCrankshaft()->m_body.index;
        system->t[shaft] += -pressureDifferential * rotorVolumeDerivative();
        return;
    }
    const double area = (bank->getBore() * bank->getBore() / 4.0) * constants::pi;
    const double v_x = system->v_x[m_piston->m_body.index];
    const double v_y = system->v_y[m_piston->m_body.index];

    const double v_s =
        v_x * bank->getDx() + v_y * bank->getDy();

    const double pressureDifferential = m_system.pressure() - crankcasePressure();
    const double force = -area * pressureDifferential;

    if (std::isnan(force) || std::isinf(force)) {
        assert(false);
    }

    constexpr double limit = 1E-3;
    const double abs_v_s = std::fmin(std::abs(v_s), limit);
    const double attenuation = abs_v_s / limit;

    // An engine with the mean-effective-pressure friction model has it all on the crank
    const double F = m_engine->getFriction().enabled ? 0.0 : calculateFrictionForce(v_s) * attenuation;
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
