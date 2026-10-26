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

            // Diesel: fuel injected into the cylinder lights itself after an ignition delay
            // (Hardenberg-Hase, by cetane number), burning in a premixed spike and then as fast
            // as it mixes. The intake carries plain air.
            bool compressionIgnition = false;
            double cetaneNumber = 50;
            // Diesel unevenness: each injector delivers a little more or less than the others
            // (standard deviation, fixed per cylinder), and every shot varies in fuel and
            // ignition delay (standard deviations). Without it every firing is identical and the
            // engine sounds like a tone generator.
            // A charge with more burned gas than this in it won't light (0: no limit): a
            // two-stroke at idle four-strokes, firing every other turn or so (ring-ding). The
            // limit varies firing to firing by misfireSpread (log-normal).
            double misfireResidual = 0.0;
            double misfireSpread = 0.15;
            double injectorSpread = 0.04;
            double shotVariation = 0.03;
            // Small deliveries scatter more: the shot variation at no load, blending down to
            // shotVariation at full fuel (0: the same throughout)
            double idleShotVariation = 0.0;
            double delayVariation = 0.15;
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
        inline bool isCompressionIgnition() const { return m_compressionIgnition; }
        inline double getCetaneNumber() const { return m_cetaneNumber; }
        inline double getInjectorSpread() const { return m_injectorSpread; }
        inline double getMisfireResidual() const { return m_misfireResidual; }
        inline double getMisfireSpread() const { return m_misfireSpread; }
        inline double getShotVariation() const { return m_shotVariation; }
        // At a fueling share (0-1)
        double shotVariationAt(double share) const {
            if (m_idleShotVariation <= 0) return m_shotVariation;
            const double s = share < 0 ? 0 : (share > 1 ? 1 : share);
            return m_idleShotVariation + (m_shotVariation - m_idleShotVariation) * s;
        }
        inline double getDelayVariation() const { return m_delayVariation; }

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
        bool m_compressionIgnition = false;
        double m_cetaneNumber = 50;
        double m_injectorSpread = 0.04;
        double m_misfireResidual = 0.0;
        double m_misfireSpread = 0.15;
        double m_shotVariation = 0.03;
        double m_idleShotVariation = 0.0;
        double m_delayVariation = 0.15;
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
