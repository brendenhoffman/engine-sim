#include "forced_induction.h"

#include "engine.h"
#include "exhaust_system.h"
#include "intake.h"
#include "units.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr double MolarMass = units::AirMolecularMass;
    constexpr double SpecificR = constants::R / units::AirMolecularMass;
    constexpr double AmbientT = 298.15;
    constexpr double Gamma0 = 1.4;
    constexpr double Cp0 = Gamma0 / (Gamma0 - 1) * SpecificR;   // Intake air, J/(kg K)
    constexpr double AmbientDensity = ForcedInduction::Ambient / (SpecificR * AmbientT);

    // Flow constants in series: 1/k^2 = 1/k1^2 + 1/k2^2
    double series(double k1, double k2) {
        if (k1 <= 0 || k2 <= 0) return 0;
        return 1 / std::sqrt(1 / (k1 * k1) + 1 / (k2 * k2));
    }

    GasSystem::Mix exhaustAir() {
        GasSystem::Mix mix;
        mix.p_inert = 0.75;
        mix.p_o2 = 0.25;
        return mix;
    }
}

void ForcedInduction::initialize(const Parameters &params, const std::vector<Stage> &stages, const GasSystem::Mix &mix) {
    m_params = params;
    m_stages = stages;
    m_state.assign(stages.size(), StageState());
    m_freshMix = mix;
    m_charge.initialize(Ambient, params.chargeVolume, AmbientT, mix);
    const double width = std::cbrt(params.chargeVolume);
    m_charge.setGeometry(width, width, 1.0, 0.0);
    m_wastegate = m_bov = 0;

    auto reference = [](double k) {
        double limit, rate;
        GasSystem::chokedConstantsAt(Gamma0, &limit, &rate);
        return std::max(1e-6, MolarMass * GasSystem::flowRate(k, 2 * Ambient, Ambient, AmbientT, AmbientT, Gamma0, limit, rate));
    };
    m_bovReference = reference(params.bovFlow);
    m_wastegateReference = reference(params.wastegateFlow);

    bool turbine = false;
    for (const Stage &s : m_stages) turbine = turbine || s.turbo;
    if (turbine && m_params.exhaust != nullptr) m_params.exhaust->setOutletExternal(true);
}

double ForcedInduction::compressorRise(const Stage &s, double speed, double massFlow, double *work) const {
    // Moore-Greitzer cubic: shut-off rise psi0 at no flow, a peak of psi0 + 2H at the surge line
    // (x = 2), falling through zero at choke (x = 4) and climbing for reverse flow. A real wheel
    // makes most of its pressure even with no flow, which a check valve needs to open.
    const double U = speed * s.compressorRadius;
    const double headRatio = 1 + s.compressorEfficiency * s.slip * U * U / (Cp0 * AmbientT);
    const double peak = Ambient * (std::pow(headRatio, Gamma0 / (Gamma0 - 1)) - 1);
    const double psi0 = 0.8 * peak, H = 0.1 * peak;
    const double choke = std::max(s.compressorFlow * speed / s.maxSpeed, 0.02 * s.compressorFlow);
    const double W = choke / 4.0;
    const double x = massFlow / W;
    const double y = std::clamp(x - 1, -4.0, 4.0);
    const double rise = psi0 + H * (1 + 1.5 * y - 0.5 * y * y * y);

    if (work != nullptr) {
        if (massFlow >= 0) {
            const double pr = 1 + std::max(rise, 0.0) / Ambient;
            const double ideal = Cp0 * AmbientT * (std::pow(pr, (Gamma0 - 1) / Gamma0) - 1);
            const double efficiency = s.compressorEfficiency * std::clamp(1 - std::pow((x - 2.6) / 2.4, 2), 0.3, 1.0);
            *work = std::max(ideal / efficiency, 0.2 * s.slip * U * U);
        }
        else {
            *work = 0.5 * s.slip * U * U; // Churning gas pushed back through it
        }
    }
    return rise;
}

void ForcedInduction::process(double dt, Engine *engine, Intake *intake) {
    const double crankSpeed = std::abs(engine->getSpeed());
    const double p2 = m_charge.pressure();
    GasSystem *collector = (m_params.exhaust != nullptr) ? m_params.exhaust->getSystem() : nullptr;
    const double outletK = (m_params.exhaust != nullptr) ? m_params.exhaust->getOutletFlowRate() : 0;
    double exhaustOut = 0;
    m_crankLoad = 0;

    for (size_t i = 0; i < m_stages.size(); ++i) {
        const Stage &s = m_stages[i];
        StageState &st = m_state[i];

        if (!s.turbo) st.speed = s.driveRatio * crankSpeed;
        // Sequential: the exhaust control valve opens progressively over the last stretch
        // before the changeover, spooling the second turbo hard while its compressor still
        // pumps round its recirculation path; the charge control valve then opens once the
        // second compressor can match the charge pressure (no hole while it catches up)
        double share = 1.0;
        if (s.sequentialSpeed > 0) {
            const double band = 0.12 * s.sequentialSpeed;
            const double x = std::clamp((crankSpeed - (s.sequentialSpeed - band)) / band, 0.0, 1.0);
            const double preopen = std::max(s.prespool, 0.4);
            const double shutOff = compressorRise(s, st.speed, 0, nullptr);
            if (crankSpeed < 0.92 * s.sequentialSpeed) st.engaged = false;
            else if (crankSpeed > s.sequentialSpeed && shutOff >= 0.95 * (p2 - Ambient)) st.engaged = true;
            share = st.engaged ? 1.0 : s.prespool + (preopen - s.prespool) * x * x * (3 - 2 * x);
        }

        // The gas in the compressor's ducting, accelerated by what the wheel makes over what
        // the charge holds
        if (s.displacement > 0) {
            // Positive displacement: a fixed volume of ambient air per turn, less the leak back
            const double dp = p2 - Ambient;
            const double ideal = AmbientDensity * s.displacement * st.speed / (2 * constants::pi);
            const double leak = s.leakage * std::sqrt(2 * AmbientDensity * std::abs(dp)) * (dp > 0 ? 1.0 : -1.0);
            st.massFlow = ideal - leak;
            const double pr = std::max(p2 / Ambient, 1.0);
            const double work = Cp0 * AmbientT * (std::pow(pr, (Gamma0 - 1) / Gamma0) - 1) / s.compressorEfficiency;
            const double dn = st.massFlow * dt / MolarMass;
            if (dn > 0) {
                double T = AmbientT + work / Cp0;
                T -= m_params.intercooler * (T - AmbientT);
                m_charge.gainN(dn, m_charge.energyPerMol(T, m_freshMix) + constants::R * T, m_freshMix);
            }
            else if (dn < 0) {
                m_charge.loseN(std::min(-dn, 0.5 * m_charge.n()), m_charge.enthalpyPerMol());
            }
            // It does that work on everything it moves, leaked or not
            st.compressorTorque = (st.speed > 1.0) ? ideal * work / st.speed : 0.0;
            m_crankLoad += s.driveRatio * (st.compressorTorque + s.friction * st.speed);
            continue;
        }

        // A sequential stage that's not brought in yet pumps round its recirculation path
        // (against nothing), its charge control valve shut
        const bool recirculating = s.sequentialSpeed > 0 && !st.engaged;
        double work = 0;
        const double rise = compressorRise(s, st.speed, st.massFlow, &work);
        const double loss = 0.5 * st.massFlow * std::abs(st.massFlow) / (2 * AmbientDensity * s.ductArea * s.ductArea);
        const double against = recirculating ? 0.0 : p2 - Ambient;
        st.massFlow += dt * (s.ductArea / s.ductLength) * (rise - against - loss);
        st.massFlow = std::clamp(st.massFlow, -2 * s.compressorFlow, 2 * s.compressorFlow);
        if (s.sequentialSpeed > 0) st.massFlow = std::max(st.massFlow, 0.0); // Check valve

        const double dn = recirculating ? 0.0 : st.massFlow * dt / MolarMass;
        if (dn > 0) {
            double T = AmbientT + work / Cp0;
            T -= m_params.intercooler * (T - AmbientT);
            m_charge.gainN(dn, m_charge.energyPerMol(T, m_freshMix) + constants::R * T, m_freshMix);
        }
        else if (dn < 0) {
            const double out = std::min(-dn, 0.5 * m_charge.n());
            m_charge.loseN(out, m_charge.enthalpyPerMol());
        }
        st.compressorTorque = (st.speed > 1.0) ? std::abs(st.massFlow) * work / st.speed : 0.0;

        if (s.turbo) {
            st.turbineTorque = 0;
            st.turbineFlow = 0;
            if (collector != nullptr && s.turbineFlow > 0) {
                const double kTurbine = s.turbineFlow * share;
                const double p3 = collector->pressure();
                const double T3 = collector->temperature();
                const double moles = collector->flow(series(kTurbine, outletK), dt, Ambient, AmbientT, exhaustAir());
                exhaustOut += moles;
                if (moles > 0 && p3 > Ambient && kTurbine > 0) {
                    st.turbineFlow = moles * MolarMass / dt;
                    // The turbine takes its share of the drop to ambient (orifices in series
                    // split it by 1/k^2)
                    const double wt = 1 / (kTurbine * kTurbine), wo = 1 / (outletK * outletK);
                    const double p4 = p3 - (p3 - Ambient) * wt / (wt + wo);
                    const double g = collector->heatCapacityRatio();
                    const double cp = g / (g - 1) * SpecificR;
                    const double cs = std::sqrt(std::max(0.0, 2 * cp * T3 * (1 - std::pow(p4 / p3, (g - 1) / g))));
                    const double b = s.turbineEfficiency / 0.98, a = 1.4 * b;
                    st.turbineTorque = st.turbineFlow * s.turbineRadius * (a * cs - b * st.speed * s.turbineRadius);
                }
            }
            st.speed += dt * (st.turbineTorque - st.compressorTorque - s.friction * st.speed) / s.inertia;
            st.speed = std::clamp(st.speed, 0.0, 1.2 * s.maxSpeed);
        }
        else {
            m_crankLoad += s.driveRatio * (st.compressorTorque + s.friction * st.speed);
        }
    }

    // Wastegate: opens as the boost passes its target
    m_wastegateFlow = 0;
    if (collector != nullptr && m_params.boost > 0 && m_params.wastegateFlow > 0) {
        const double target = std::clamp((p2 - Ambient - m_params.boost) / m_params.wastegateRange, 0.0, 1.0);
        m_wastegate += (target - m_wastegate) * std::min(1.0, dt / 0.05);
        if (m_wastegate > 1e-4) {
            const double moles = collector->flow(series(m_params.wastegateFlow * m_wastegate, outletK), dt, Ambient, AmbientT, exhaustAir());
            exhaustOut += moles;
            m_wastegateFlow = std::max(0.0, moles * MolarMass / dt);
        }
    }
    if (m_params.exhaust != nullptr && m_params.exhaust->isOutletExternal()) m_params.exhaust->setExternalFlow(exhaustOut);

    // Blow-off valve: the throttle has shut on a pressurised charge
    m_bovFlow = 0;
    if (m_params.bovFlow > 0) {
        const double plenum = intake->m_system.pressure();
        const double target = std::clamp((p2 - plenum - m_params.bovPressure) / (0.5 * m_params.bovPressure), 0.0, 1.0);
        m_bov += (target - m_bov) * std::min(1.0, dt / 0.02);
        if (m_bov > 1e-4) {
            const double moles = m_charge.flow(m_params.bovFlow * m_bov, dt, Ambient, AmbientT, m_freshMix);
            m_bovFlow = std::max(0.0, moles * MolarMass / dt);
        }
    }
}

double ForcedInduction::boost() const {
    return m_charge.pressure() - Ambient;
}

ForcedInduction::Voice ForcedInduction::voice(int i) const {
    const Stage &s = m_stages[i];
    const StageState &st = m_state[i];
    Voice v;
    const double speed = std::clamp(st.speed / s.maxSpeed, 0.0, 1.2);
    const double capacity = (s.displacement > 0)
        ? AmbientDensity * s.displacement * s.maxSpeed / (2 * constants::pi)
        : s.compressorFlow;
    const double flow = st.massFlow / capacity;
    v.frequency = s.whineOrder * st.speed / (2 * constants::pi);
    v.whine = m_params.whineVolume * std::pow(speed, 1.5) * (0.3 + 0.7 * std::min(1.0, std::abs(flow)));
    v.hiss = m_params.hissVolume * std::abs(flow) * std::sqrt(speed);
    v.flutter = m_params.flutterVolume * flow;
    if (!s.turbo && s.gearOrder > 0 && s.driveRatio > 0) {
        const double crank = st.speed / s.driveRatio;
        v.gearFrequency = s.gearOrder * crank / (2 * constants::pi);
        v.gear = s.gearVolume * std::clamp(st.speed / s.maxSpeed, 0.0, 1.2) * (0.4 + 0.6 * std::min(1.0, std::abs(flow)));
    }
    if (i == 0) {
        // Relative to the flow the fully open valve passes on 1 bar
        v.bov = m_params.bovVolume * (m_params.bovVent ? 1.0 : 0.3) * std::min(2.0, m_bovFlow / m_bovReference);
        v.wastegate = m_params.wastegateVolume * std::min(2.0, m_wastegateFlow / m_wastegateReference);
    }
    return v;
}
