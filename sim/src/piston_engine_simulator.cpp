#include "piston_engine_simulator.h"
#include "forced_induction.h"
#include <cstdio>
#include <algorithm>
#include <cstdlib>

#include "constants.h"
#include "units.h"

#include <cmath>
#include <assert.h>
#include <chrono>
#include <set>

PistonEngineSimulator::PistonEngineSimulator() {
    m_engine = nullptr;
    m_transmission = nullptr;
    m_vehicle = nullptr;
    m_delayFilters = nullptr;

    m_crankConstraints = nullptr;
    m_cylinderWallConstraints = nullptr;
    m_linkConstraints = nullptr;
    m_crankshaftFrictionConstraints = nullptr;
    m_crankshaftLinks = nullptr;

    m_exhaustFlowStagingBuffer = nullptr;

    m_derivativeFilter.m_dt = 1.0;
    m_fluidSimulationSteps = 8;
}

PistonEngineSimulator::~PistonEngineSimulator() {
    assert(m_crankConstraints == nullptr);
    assert(m_cylinderWallConstraints == nullptr);
    assert(m_linkConstraints == nullptr);
    assert(m_crankshaftFrictionConstraints == nullptr);
    assert(m_exhaustFlowStagingBuffer == nullptr);
    assert(m_delayFilters == nullptr);
}

void PistonEngineSimulator::loadSimulation(Engine *engine, Vehicle *vehicle, Transmission *transmission) {
    Simulator::loadSimulation(engine, vehicle, transmission);

    m_engine = engine;
    m_vehicle = vehicle;
    m_transmission = transmission;

    const int crankCount = m_engine->getCrankshaftCount();
    const int cylinderCount = m_engine->getCylinderCount();
    const int linkCount = cylinderCount * 2;

    if (crankCount <= 0) return;

    m_crankConstraints = new atg_scs::FixedPositionConstraint[crankCount];
    m_cylinderWallConstraints = new atg_scs::LineConstraint[cylinderCount];
    m_linkConstraints = new atg_scs::LinkConstraint[linkCount];
    m_crankshaftFrictionConstraints = new atg_scs::RotationFrictionConstraint[crankCount];
    m_crankshaftLinks = new atg_scs::ClutchConstraint[crankCount - 1];
    m_delayFilters = new DelayFilter[cylinderCount];

    const double ks = 5000;
    const double kd = 10;

    for (int i = 0; i < crankCount; ++i) {
        Crankshaft *outputShaft = m_engine->getCrankshaft(0);
        Crankshaft *crankshaft = m_engine->getCrankshaft(i);

        m_crankConstraints[i].setBody(&crankshaft->m_body);
        m_crankConstraints[i].setWorldPosition(
            crankshaft->getPosX(),
            crankshaft->getPosY());
        m_crankConstraints[i].setLocalPosition(0.0, 0.0);
        m_crankConstraints[i].m_kd = kd;
        m_crankConstraints[i].m_ks = ks;

        crankshaft->m_body.p_x = crankshaft->getPosX();
        crankshaft->m_body.p_y = crankshaft->getPosY();
        crankshaft->m_body.theta = 0;
        crankshaft->m_body.m =
            crankshaft->getMass() + crankshaft->getFlywheelMass();
        crankshaft->m_body.I = crankshaft->getMomentOfInertia();

        m_crankshaftFrictionConstraints[i].m_minTorque = -crankshaft->getFrictionTorque();
        m_crankshaftFrictionConstraints[i].m_maxTorque = crankshaft->getFrictionTorque();
        m_crankshaftFrictionConstraints[i].setBody(&m_engine->getCrankshaft(i)->m_body);

        m_system->addRigidBody(&m_engine->getCrankshaft(i)->m_body);
        m_system->addConstraint(&m_crankConstraints[i]);
        m_system->addConstraint(&m_crankshaftFrictionConstraints[i]);

        if (crankshaft != outputShaft) {
            atg_scs::ClutchConstraint *crankLink = &m_crankshaftLinks[i - 1];
            crankLink->setBody1(&outputShaft->m_body);
            crankLink->setBody2(&crankshaft->m_body);

            m_system->addConstraint(crankLink);
        }
    }

    m_transmission->addToSystem(m_system, &m_vehicleMass, m_vehicle, m_engine);
    m_vehicle->addToSystem(m_system, &m_vehicleMass);

    m_vehicleDrag.initialize(&m_vehicleMass, m_vehicle);
    m_system->addConstraint(&m_vehicleDrag);

    m_vehicleMass.reset();
    m_vehicleMass.m = 1.0;
    m_vehicleMass.I = 1.0;
    m_system->addRigidBody(&m_vehicleMass);

    for (int i = 0; i < cylinderCount; ++i) {
        Piston *piston = m_engine->getPiston(i);
        ConnectingRod *connectingRod = piston->getRod();

        CylinderBank *bank = piston->getCylinderBank();
        if (bank->isRotor()) {
            // Rotor faces have no piston or rod to simulate; the chamber turns the shaft
            m_system->addForceGenerator(m_engine->getChamber(i));
            continue;
        }
        const double dx = std::cos(bank->getAngle() + constants::pi / 2);
        const double dy = std::sin(bank->getAngle() + constants::pi / 2);

        m_cylinderWallConstraints[i].setBody(&piston->m_body);
        m_cylinderWallConstraints[i].m_dx = dx;
        m_cylinderWallConstraints[i].m_dy = dy;
        m_cylinderWallConstraints[i].m_local_x = 0.0;
        m_cylinderWallConstraints[i].m_local_y = piston->getWristPinLocation();
        m_cylinderWallConstraints[i].m_p0_x = bank->getX();
        m_cylinderWallConstraints[i].m_p0_y = bank->getY();
        m_cylinderWallConstraints[i].m_ks = ks;
        m_cylinderWallConstraints[i].m_kd = kd;

        piston->setCylinderConstraint(&m_cylinderWallConstraints[i]);

        m_linkConstraints[i * 2 + 0].setBody1(&connectingRod->m_body);
        m_linkConstraints[i * 2 + 0].setBody2(&piston->m_body);
        m_linkConstraints[i * 2 + 0]
            .setLocalPosition1(0.0, connectingRod->getLittleEndLocal());
        m_linkConstraints[i * 2 + 0].setLocalPosition2(0.0, piston->getWristPinLocation());
        m_linkConstraints[i * 2 + 0].m_ks = ks;
        m_linkConstraints[i * 2 + 0].m_kd = kd;

        double journal_x = 0.0, journal_y = 0.0;
        if (connectingRod->getMasterRod() == nullptr) {
            Crankshaft *crankshaft = connectingRod->getCrankshaft();
            crankshaft->getRodJournalPositionLocal(
                connectingRod->getJournal(),
                &journal_x,
                &journal_y);
            m_linkConstraints[i * 2 + 1].setBody2(&crankshaft->m_body);
        }
        else {
            connectingRod->getMasterRod()->getRodJournalPositionLocal(
                connectingRod->getJournal(),
                &journal_x,
                &journal_y);
            m_linkConstraints[i * 2 + 1].setBody2(&connectingRod->getMasterRod()->m_body);
        }

        m_linkConstraints[i * 2 + 1].setBody1(&connectingRod->m_body);
        m_linkConstraints[i * 2 + 1]
            .setLocalPosition1(0.0, connectingRod->getBigEndLocal());
        m_linkConstraints[i * 2 + 1]
            .setLocalPosition2(journal_x, journal_y);
        m_linkConstraints[i * 2 + 1].m_ks = ks;
        m_linkConstraints[i * 2 + 0].m_kd = kd;

        piston->m_body.m = piston->getMass();
        piston->m_body.I = 1.0;

        connectingRod->m_body.m = connectingRod->getMass();
        connectingRod->m_body.I = connectingRod->getMomentOfInertia();

        m_system->addRigidBody(&piston->m_body);
        m_system->addRigidBody(&connectingRod->m_body);
        m_system->addConstraint(&m_linkConstraints[i * 2 + 0]);
        m_system->addConstraint(&m_linkConstraints[i * 2 + 1]);
        m_system->addConstraint(&m_cylinderWallConstraints[i]);
        m_system->addForceGenerator(m_engine->getChamber(i));
    }

    m_dyno.connectCrankshaft(m_engine->getOutputCrankshaft());
    m_system->addConstraint(&m_dyno);

    m_crankTorque.connect(&m_engine->getOutputCrankshaft()->m_body);
    m_crankTorque.torque = 0;
    m_system->addForceGenerator(&m_crankTorque);
    m_accessoryTorque.connect(&m_engine->getOutputCrankshaft()->m_body);
    m_accessoryTorque.torque = 0;
    m_system->addForceGenerator(&m_accessoryTorque);
    m_frictionTorque.connect(&m_engine->getOutputCrankshaft()->m_body);
    m_frictionTorque.torque = 0;
    m_system->addForceGenerator(&m_frictionTorque);
    m_peakCylinderPressure = units::pressure(1.0, units::atm);

    m_starterMotor.connectCrankshaft(m_engine->getOutputCrankshaft());
    m_starterMotor.m_maxTorque = m_engine->getStarterTorque();
    m_starterMotor.m_rotationSpeed = -m_engine->getStarterSpeed();
    m_system->addConstraint(&m_starterMotor);

    placeAndInitialize();
    initializeSynthesizer();
}

double PistonEngineSimulator::getAverageOutputSignal() const {
    double sum = 0.0;
    for (int i = 0; i < m_engine->getExhaustSystemCount(); ++i) {
        sum += m_engine->getExhaustSystem(i)->getSystem()->pressure();
    }

    return sum / m_engine->getExhaustSystemCount();
}

void PistonEngineSimulator::placeAndInitialize() {
    const int cylinderCount = m_engine->getCylinderCount();
    for (int i = 0; i < cylinderCount; ++i) {
        if (m_engine->getPiston(i)->getCylinderBank()->isRotor()) continue;
        ConnectingRod *rod = m_engine->getConnectingRod(i);

        if (rod->getRodJournalCount() != 0) {
            placeCylinder(i);
        }
    }

    for (int i = 0; i < cylinderCount; ++i) {
        if (m_engine->getPiston(i)->getCylinderBank()->isRotor()) continue;
        placeCylinder(i);
    }

    for (int i = 0; i < cylinderCount; ++i) {
        m_engine->getChamber(i)->m_system.initialize(
            units::pressure(1.0, units::atm),
            m_engine->getChamber(i)->getVolume(),
            units::celcius(25.0)
        );

        Piston *piston = m_engine->getChamber(i)->getPiston();
        CylinderHead *head = m_engine->getChamber(i)->getCylinderHead();
        ExhaustSystem *exhaust = head->getExhaustSystem(piston->getCylinderIndex());
        const double exhaustLength = m_engine->getChamber(i)->exhaustSoundPathLength();
        const double speedOfSound = 343.0 * units::m / units::sec;
        const double delay = exhaustLength / speedOfSound;
        m_delayFilters[i].initialize(delay, 10000.0);
    }

    // Diesels: plain air in the intakes, the fuel goes into the cylinders
    if (m_engine->getFuel()->isCompressionIgnition()) {
        for (int i = 0; i < m_engine->getIntakeCount(); ++i) {
            Intake *intake = m_engine->getIntake(i);
            intake->setDirectInjection(true);
            // Clear the fuel-air mix they were set up with
            intake->m_system.changeMix(intake->freshMix());
            if (ForcedInduction *fi = intake->getForcedInduction()) fi->charge()->changeMix(intake->freshMix());
        }
    }
    m_lastChamberPressure.assign(cylinderCount, units::pressure(1.0, units::atm));
    m_drone.assign(m_engine->getExhaustSystemCount(), Drone());

    // Every volume and pipe has to share a gas model: a mole crossing between two models would
    // change temperature on the way
    if (m_engine->getRealGas()) {
        const GasSystem::Properties *real = &GasSystem::Properties::Real;
        for (int i = 0; i < cylinderCount; ++i) m_engine->getChamber(i)->setGasProperties(real);
        for (int i = 0; i < m_engine->getIntakeCount(); ++i) m_engine->getIntake(i)->setGasProperties(real);
        for (int i = 0; i < m_engine->getExhaustSystemCount(); ++i) m_engine->getExhaustSystem(i)->setGasProperties(real);
    }

    m_engine->getIgnitionModule()->reset();

    m_exhaustFlowStagingBuffer = new double[m_engine->getExhaustSystemCount() + 1
        + Synthesizer::VoiceInputs * m_engine->getCompressorStageCount()];
}

void PistonEngineSimulator::placeCylinder(int i) {
    ConnectingRod *rod = m_engine->getConnectingRod(i);
    Piston *piston = m_engine->getPiston(i);
    CylinderBank *bank = piston->getCylinderBank();

    double p_x, p_y;
    if (rod->getMasterRod() != nullptr) {
        rod->getMasterRod()->getRodJournalPositionGlobal(rod->getJournal(), &p_x, &p_y);
    }
    else {
        rod->getCrankshaft()->getRodJournalPositionGlobal(rod->getJournal(), &p_x, &p_y);
    }

    // (bank->m_x + bank->m_dx * s - p_x)^2 + (bank->m_y + bank->m_dy * s - p_y)^2 = (rod->m_length)^2
    const double a = bank->getDx() * bank->getDx() + bank->getDy() * bank->getDy();
    const double b = -2 * bank->getDx() * (p_x - bank->getX()) - 2 * bank->getDy() * (p_y - bank->getY());
    const double c =
        (p_x - bank->getX()) * (p_x - bank->getX())
        + (p_y - bank->getY()) * (p_y - bank->getY())
        - rod->getLength() * rod->getLength();

    const double det = b * b - 4 * a * c;
    if (det < 0) return;

    const double sqrt_det = std::sqrt(det);
    const double s0 = (-b + sqrt_det) / (2 * a);
    const double s1 = (-b - sqrt_det) / (2 * a);

    const double s = std::max(s0, s1);
    if (s < 0) return;

    const double e_x = s * bank->getDx() + bank->getX();
    const double e_y = s * bank->getDy() + bank->getY();

    const double theta = ((e_y - p_y) > 0)
        ? std::acos((e_x - p_x) / rod->getLength())
        : 2 * constants::pi - std::acos((e_x - p_x) / rod->getLength());
    rod->m_body.theta = theta - constants::pi / 2;

    double cl_x, cl_y;
    rod->m_body.localToWorld(0, rod->getBigEndLocal(), &cl_x, &cl_y);
    rod->m_body.p_x += p_x - cl_x;
    rod->m_body.p_y += p_y - cl_y;

    piston->m_body.p_x = e_x;
    piston->m_body.p_y = e_y;
    piston->m_body.theta = bank->getAngle() + constants::pi;
}

void PistonEngineSimulator::simulateStep_() {
    const double timestep = getTimestep();
    IgnitionModule *im = m_engine->getIgnitionModule();
    im->update(timestep);
    if (m_engine->getFuel()->isCompressionIgnition() || m_engine->getJakeBrakeFlow() > 0) {
        m_engine->updateDieselGovernor(timestep);
    }

    const int cylinderCount = m_engine->getCylinderCount();
    for (int i = 0; i < cylinderCount; ++i) {
        if (im->getIgnitionEvent(i)) {
            m_engine->getChamber(i)->ignite();
        }

        m_engine->getChamber(i)->update(timestep);
    }

    for (int i = 0; i < cylinderCount; ++i) {
        m_engine->getChamber(i)->resetLastTimestepExhaustFlow();
        m_engine->getChamber(i)->resetLastTimestepIntakeFlow();
    }

    const int exhaustSystemCount = m_engine->getExhaustSystemCount();
    const int intakeCount = m_engine->getIntakeCount();
    const double fluidTimestep = timestep / m_fluidSimulationSteps;
    double accessoryLoad = 0;
    for (int i = 0; i < m_fluidSimulationSteps; ++i) {
        for (int j = 0; j < intakeCount; ++j) {
            Intake *intake = m_engine->getIntake(j);
            if (ForcedInduction *fi = intake->getForcedInduction()) {
                fi->process(fluidTimestep, m_engine, intake);
                accessoryLoad += fi->crankLoad();
            }
        }

        for (int j = 0; j < exhaustSystemCount; ++j) {
            m_engine->getExhaustSystem(j)->process(fluidTimestep);
        }

        for (int j = 0; j < intakeCount; ++j) {
            m_engine->getIntake(j)->process(fluidTimestep);
            m_engine->getIntake(j)->m_flowRate += m_engine->getIntake(j)->m_flow;
        }

        for (int j = 0; j < cylinderCount; ++j) {
            m_engine->getChamber(j)->flow(fluidTimestep);
        }
    }

    im->resetIgnitionEvents();

    // Superchargers' drag, against the crank's rotation, on the next step
    const double crankSpeed = m_engine->getOutputCrankshaft()->m_body.v_theta;
    m_accessoryTorque.torque = (crankSpeed > 0 ? -1.0 : 1.0) * accessoryLoad / m_fluidSimulationSteps;

    // Friction as a mean effective pressure: the work it takes per cycle spread over the cycle.
    // The peak cylinder pressure is held for about a cycle so it reads each firing's peak.
    const Engine::Parameters::Friction &friction = m_engine->getFriction();
    if (friction.enabled) {
        double pressure = 0;
        for (int i = 0; i < cylinderCount; ++i) pressure = std::max(pressure, m_engine->getChamber(i)->m_system.pressure());
        const double omega = std::abs(crankSpeed);
        const double cycle = m_engine->getOutputCrankshaft()->getCycleLength();
        const double hold = std::exp(-timestep * std::max(omega, 10.0) / cycle);
        m_peakCylinderPressure = std::max(pressure, units::pressure(1.0, units::atm) + hold * (m_peakCylinderPressure - units::pressure(1.0, units::atm)));
        const double pistonSpeed = m_engine->getStroke() * omega / constants::pi;
        const double fmep = friction.constant + friction.peakPressure * m_peakCylinderPressure
            + friction.speed * pistonSpeed + friction.speed2 * pistonSpeed * pistonSpeed;
        const double torque = fmep * m_engine->getDisplacement() / cycle;
        // Smoothly through zero speed, so a stopped crank isn't kicked either way
        m_frictionTorque.torque = -torque * std::tanh(crankSpeed / 2.0);
    }
}

double PistonEngineSimulator::getTotalExhaustFlow() const {
    double totalFlow = 0.0;
    for (int i = 0; i < m_engine->getCylinderCount(); ++i) {
        totalFlow += m_engine->getChamber(i)->getLastTimestepExhaustFlow();
    }

    return totalFlow;
}

void PistonEngineSimulator::endFrame() {
    Simulator::endFrame();

    if (m_engine == nullptr) {
        return;
    }

    const double frameTimestep = simulationSteps() * getTimestep();
    const int cylinderCount = m_engine->getCylinderCount();
    for (int i = 0; i < m_engine->getIntakeCount(); ++i) {
        m_engine->getIntake(i)->m_flowRate /= frameTimestep;
    }
}

void PistonEngineSimulator::destroy() {
    if (m_system != nullptr) m_system->reset();

    if (m_crankConstraints != nullptr) delete[] m_crankConstraints;
    if (m_cylinderWallConstraints != nullptr) delete[] m_cylinderWallConstraints;
    if (m_linkConstraints != nullptr) delete[] m_linkConstraints;
    if (m_crankshaftFrictionConstraints != nullptr) delete[] m_crankshaftFrictionConstraints;
    if (m_crankshaftLinks != nullptr) delete[] m_crankshaftLinks;
    if (m_exhaustFlowStagingBuffer != nullptr) delete[] m_exhaustFlowStagingBuffer;
    if (m_system != nullptr) delete m_system;
    if (m_delayFilters != nullptr) delete[] m_delayFilters;

    m_crankConstraints = nullptr;
    m_cylinderWallConstraints = nullptr;
    m_linkConstraints = nullptr;
    m_crankshaftFrictionConstraints = nullptr;
    m_crankshaftLinks = nullptr;
    m_exhaustFlowStagingBuffer = nullptr;
    m_system = nullptr;

    m_vehicle = nullptr;
    m_transmission = nullptr;
    m_engine = nullptr;
    m_delayFilters = nullptr;

    // Dyno samples and synthesizer buffers belong to the base class
    Simulator::destroy();
}

void PistonEngineSimulator::writeToSynthesizer() {
    const int exhaustSystemCount = m_engine->getExhaustSystemCount();
    const bool block = m_engine->getCombustionNoise() > 0;
    for (int i = 0; i < exhaustSystemCount + (block ? 1 : 0); ++i) {
        m_exhaustFlowStagingBuffer[i] = 0;
    }

    const double attenuation = std::min(std::abs(filteredEngineSpeed()), 40.0) / 40.0;
    const double attenuation_3 = attenuation * attenuation * attenuation;

    const double timestep = getTimestep();
    const int cylinderCount = m_engine->getCylinderCount();
    for (int i = 0; i < cylinderCount; ++i) {
        Piston *piston = m_engine->getPiston(i);
        CylinderBank *bank = piston->getCylinderBank();
        CylinderHead *head = m_engine->getHead(bank->getIndex());
        ExhaustSystem *exhaust = head->getExhaustSystem(piston->getCylinderIndex());
        CombustionChamber *chamber = m_engine->getChamber(i);

        const double exhaustLength =
            head->getHeaderPrimaryLength(piston->getCylinderIndex())
            + exhaust->getLength();

        double exhaustFlow = attenuation_3 * 1600 * chamber->exhaustSoundSignal();

        // Combustion noise: the cylinder pressure's rise each step (scaled to a 10 kHz step),
        // heard through the block rather than the exhaust, so without its delay
        double knock = 0;
        if (m_engine->getCombustionNoise() > 0 && i < static_cast<int>(m_lastChamberPressure.size())) {
            const double p = chamber->m_system.pressure();
            knock = 1000 * m_engine->getCombustionNoise() * attenuation_3 * (p - m_lastChamberPressure[i]) * (timestep * 10000);
            m_lastChamberPressure[i] = p;
        }

        const double delayedExhaustPulse =
            m_delayFilters[i].fast_f(exhaustFlow);

        ExhaustSystem *exhaustSystem = head->getExhaustSystem(piston->getCylinderIndex());
        m_exhaustFlowStagingBuffer[exhaustSystem->getIndex()] +=
            head->getSoundAttenuation(piston->getCylinderIndex())
            * (exhaustSystem->getAudioVolume() * delayedExhaustPulse / cylinderCount)
            * (1 / (exhaustLength * exhaustLength));
        if (block) m_exhaustFlowStagingBuffer[exhaustSystemCount] += knock / cylinderCount;
    }

    // Muffler drone: a two-pole low-pass following the firing frequency
    const double firing = cylinderCount * std::abs(m_engine->getOutputCrankshaft()->m_body.v_theta)
        / m_engine->getOutputCrankshaft()->getCycleLength();
    for (int i = 0; i < exhaustSystemCount && i < static_cast<int>(m_drone.size()); ++i) {
        const double k = m_engine->getExhaustSystem(i)->getDroneFilter();
        if (k <= 0) continue;
        const double fs = 1.0 / timestep;
        const double fc = std::clamp(k * firing, 20.0, 0.4 * fs);
        const double w0 = 2 * constants::pi * fc / fs;
        const double alpha = std::sin(w0) / (2 * 0.7071);
        const double cw = std::cos(w0), a0 = 1 + alpha;
        const double b0 = (1 - cw) / 2 / a0, b1 = (1 - cw) / a0, a1 = -2 * cw / a0, a2 = (1 - alpha) / a0;
        Drone &d = m_drone[i];
        const double x = m_exhaustFlowStagingBuffer[i];
        const double y = b0 * x + b1 * d.x1 + b0 * d.x2 - a1 * d.y1 - a2 * d.y2;
        d.x2 = d.x1; d.x1 = x; d.y2 = d.y1; d.y1 = y;
        m_exhaustFlowStagingBuffer[i] = y;
    }

    static const bool signalTrace = std::getenv("ENGINE_SIM_SIGNAL_TRACE") != nullptr;
    if (signalTrace) {
        static double ex = 0, bl = 0; static int n = 0;
        ex += m_exhaustFlowStagingBuffer[0] * m_exhaustFlowStagingBuffer[0];
        if (block) bl += m_exhaustFlowStagingBuffer[exhaustSystemCount] * m_exhaustFlowStagingBuffer[exhaustSystemCount];
        if (++n == 20000) { std::fprintf(stderr, "signal rms: exhaust %.3g block %.3g\n", std::sqrt(ex / n), std::sqrt(bl / n)); ex = bl = 0; n = 0; }
    }

    // Turbo and supercharger voices
    double *voice = m_exhaustFlowStagingBuffer + exhaustSystemCount + (block ? 1 : 0);
    for (int i = 0; i < m_engine->getIntakeCount(); ++i) {
        const ForcedInduction *fi = m_engine->getIntake(i)->getForcedInduction();
        if (fi == nullptr) continue;
        for (int j = 0; j < fi->stageCount(); ++j) {
            const ForcedInduction::Voice v = fi->voice(j);
            voice[0] = v.frequency;
            voice[1] = v.whine;
            voice[2] = v.hiss;
            voice[3] = v.flutter;
            voice[4] = v.bov;
            voice[5] = v.wastegate;
            voice[6] = v.gearFrequency;
            voice[7] = v.gear;
            voice += Synthesizer::VoiceInputs;
        }
    }

    synthesizer().writeInput(m_exhaustFlowStagingBuffer);
    synthesizer().setLevelerHold(m_starterMotor.m_enabled || std::abs(m_engine->getRpm()) < 300);
}
