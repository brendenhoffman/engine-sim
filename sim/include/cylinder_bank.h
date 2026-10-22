#ifndef ATG_ENGINE_SIM_CYLINDER_BANK_H
#define ATG_ENGINE_SIM_CYLINDER_BANK_H

#include "part.h"

#include "crankshaft.h"

class CylinderBank {
    public:
        struct Parameters {
            Crankshaft *crankshaft;
            double positionX;
            double positionY;
            double angle;
            double bore;
            double deckHeight;
            double displayDepth;
            int cylinderCount;
            int index;

            // A Wankel rotor housing instead of a bank of cylinders when the eccentricity is
            // non-zero. Its "cylinders" are the rotor's three faces.
            double rotorGeneratingRadius = 0;   // R: centre to apex
            double rotorEccentricity = 0;       // e: crank throw of the eccentric shaft
            double rotorWidth = 0;              // B
            double rotorPhase = 0;              // Cycle angle at which face 0 is at compression TDC
            int sparkPlugs = 1;                 // Per face (rotaries usually have a leading and a trailing plug)
            double trailingDelay = 0;           // Shaft angle the plugs after the first fire later by
        };

    public:
        CylinderBank();
        ~CylinderBank();

        void initialize(const Parameters &params);
        void destroy();

        void getPositionAboveDeck(double h, double *x, double *y) const;
        double boreSurfaceArea() const;

        inline double getAngle() const { return m_angle; }
        inline double getBore() const { return m_bore; }
        inline double getDeckHeight() const { return m_deckHeight; }
        inline int getCylinderCount() const { return m_cylinderCount; }
        inline int getIndex() const { return m_index; }
        inline double getDx() const { return m_dx; }
        inline double getDy() const { return m_dy; }
        inline double getX() const { return m_x; }
        inline double getY() const { return m_y; }
        inline double getDisplayDepth() const { return m_displayDepth; }

        // Rotor housings (see Parameters)
        inline bool isRotor() const { return m_rotorEccentricity > 0; }
        inline double getRotorGeneratingRadius() const { return m_rotorGeneratingRadius; }
        inline double getRotorEccentricity() const { return m_rotorEccentricity; }
        inline double getRotorWidth() const { return m_rotorWidth; }
        inline int getSparkPlugs() const { return m_sparkPlugs; }
        inline double getTrailingDelay() const { return m_trailingDelay; }

        // Swept volume of one rotor face: 3 sqrt(3) e R B
        double rotorFaceDisplacement() const;
        // Length of a rotor flank (apex to apex)
        double rotorFlankLength() const;
        // Cycle angle (crank angle within the engine cycle) where a face is at compression TDC
        double rotorFaceTdc(int face) const;
        // A face's swept volume above its minimum at a cycle angle, and its rate of change
        // with that angle. The volume of a face varies sinusoidally with two thirds of the
        // eccentric shaft angle: V = Vh/2 (1 - cos(2 phi / 3)).
        double rotorFaceSweptVolume(int face, double cycleAngle) const;
        double rotorFaceVolumeDerivative(int face, double cycleAngle) const;

    protected:
        double m_angle;
        double m_bore;
        double m_deckHeight;
        double m_displayDepth;
        int m_cylinderCount;
        int m_index;

        double m_dx;
        double m_dy;
        double m_x;
        double m_y;

        double m_rotorGeneratingRadius = 0;
        double m_rotorEccentricity = 0;
        double m_rotorWidth = 0;
        double m_rotorPhase = 0;
        int m_sparkPlugs = 1;
        double m_trailingDelay = 0;
};

#endif /* ATG_ENGINE_SIM_CYLINDER_BANK_H */
