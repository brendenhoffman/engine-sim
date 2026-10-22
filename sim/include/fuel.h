#ifndef ATG_ENGINE_FUEL_H
#define ATG_ENGINE_FUEL_H

#include "units.h"

#include "function.h"

#include <string>

class Fuel {
    public:
        struct Parameters {
            std::string name = "Gasoline";
            double molecularMass =
                units::mass(100.0, units::g);
            double energyDensity =
                units::energy(48.1, units::kJ) / units::mass(1.0, units::g);
            double density =
                units::mass(0.755, units::kg) / units::volume(1.0, units::L);
            double molecularAfr = 25 / 2.0;
            double burningEfficiencyRandomness = 0.5;
            double lowEfficiencyAttenuation = 0.6;
            double maxBurningEfficiency = 0.8;
            double maxTurbulenceEffect = 2.0;
            double maxDilutionEffect = 50.0;

            // How much burned gas left in the charge slows the flame: 0 ignores it (upstream),
            // 1 is the full laminar-flame correlation S_L * (1 - 2.06 x^0.77) (Metghalchi &
            // Keck), x the residual fraction. A charge whose flame can't spread misfires.
            double dilutionSensitivity = 0.0;

            // Cycle-to-cycle variation of the flame (turbulence and mixing differ every
            // firing), growing with the residual gas and fading at speed: each firing's flame
            // speed is scaled by exp(cyclic_variation * residual * min(1, 5 m/s / mean piston
            // speed) * N(0, 1)). 0: none.
            double cyclicVariation = 0.0;
            Function *turbulenceToFlameSpeedRatio = nullptr;
        };

        Fuel();
        ~Fuel();

        void initialize(const Parameters &params);

        inline double getMolecularMass() const { return m_molecularMass; }
        inline double getEnergyDensity() const { return m_energyDensity; }
        inline double getDensity() const { return m_density; }
        inline double getBurningEfficiencyRandomness() const { return m_burningEfficiencyRandomness; }
        inline double getLowEfficiencyAttenuation() const { return m_lowEfficiencyAttenuation;  }
        inline double getMaxBurningEfficiency() const { return m_maxBurningEfficiency; }
        inline double getMaxTurbulenceEffect() const { return m_maxTurbulenceEffect; }
        inline double getMaxDilutionEffect() const { return m_maxDilutionEffect; }
        inline double getDilutionSensitivity() const { return m_dilutionSensitivity; }
        inline double getCyclicVariation() const { return m_cyclicVariation; }

        // Factor (0-1) the residual gas fraction leaves of the flame speed
        double residualFlameFactor(double residualFraction) const;

        double flameSpeed(
            double turbulence,
            double molecularAfr,
            double T,
            double P,
            double firingPressure,
            double motoringPressure) const;
        virtual double laminarBurningVelocity(double molecularAfr, double T, double P) const;

        double getMolecularAfr() const { return m_molecularAfr; }

    protected:
        std::string m_name;
        double m_molecularMass;
        double m_energyDensity;
        double m_density;
        double m_molecularAfr;
        double m_maxBurningEfficiency;
        double m_burningEfficiencyRandomness;
        double m_lowEfficiencyAttenuation;
        double m_maxTurbulenceEffect;
        double m_maxDilutionEffect;
        double m_dilutionSensitivity = 0.0;
        double m_cyclicVariation = 0.0;

        Function *m_turbulenceToFlameSpeedRatio;
};

#endif /* ATG_ENGINE_FUEL_H */
