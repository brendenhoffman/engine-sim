#include "cylinder_bank.h"

#include "constants.h"

#include <cmath>

CylinderBank::CylinderBank() {
    m_angle = 0.0;
    m_bore = 0.0;
    m_deckHeight = 0.0;
    m_cylinderCount = 0;
    m_displayDepth = 0.4;
    m_index = -1;

    m_dx = m_dy = 0;
    m_x = m_y = 0;
}

CylinderBank::~CylinderBank() {
    /* void */
}

void CylinderBank::initialize(const Parameters &params) {
    m_angle = params.angle;
    m_bore = params.bore;
    m_deckHeight = params.deckHeight;
    m_cylinderCount = params.cylinderCount;

    m_dx = std::cos(m_angle + constants::pi / 2);
    m_dy = std::sin(m_angle + constants::pi / 2);

    m_x = params.positionX;
    m_y = params.positionY;

    m_displayDepth = params.displayDepth;

    m_index = params.index;

    m_rotorGeneratingRadius = params.rotorGeneratingRadius;
    m_rotorEccentricity = params.rotorEccentricity;
    m_rotorWidth = params.rotorWidth;
    m_rotorPhase = params.rotorPhase;
    m_sparkPlugs = params.sparkPlugs < 1 ? 1 : params.sparkPlugs;
    m_trailingDelay = params.trailingDelay;
}

double CylinderBank::rotorFaceDisplacement() const {
    return 3 * std::sqrt(3.0) * m_rotorEccentricity * m_rotorGeneratingRadius * m_rotorWidth;
}

double CylinderBank::rotorFlankLength() const {
    return std::sqrt(3.0) * m_rotorGeneratingRadius;
}

double CylinderBank::rotorFaceTdc(int face) const {
    // The faces follow each other a third of a rotor turn (a full shaft turn) apart
    return m_rotorPhase + face * 2 * constants::pi;
}

double CylinderBank::rotorFaceSweptVolume(int face, double cycleAngle) const {
    const double phi = cycleAngle - rotorFaceTdc(face);
    return 0.5 * rotorFaceDisplacement() * (1 - std::cos(2 * phi / 3));
}

double CylinderBank::rotorFaceVolumeDerivative(int face, double cycleAngle) const {
    const double phi = cycleAngle - rotorFaceTdc(face);
    return rotorFaceDisplacement() / 3 * std::sin(2 * phi / 3);
}

void CylinderBank::destroy() {
    /* void */
}

void CylinderBank::getPositionAboveDeck(double h, double *x, double *y) const {
    *x = m_dx * (m_deckHeight + h) + m_x;
    *y = m_dy * (m_deckHeight + h) + m_y;
}

double CylinderBank::boreSurfaceArea() const {
    return constants::pi * m_bore * m_bore / 4.0;
}
