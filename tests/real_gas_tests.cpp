#include <gtest/gtest.h>

#include "gas_system.h"
#include "pipe.h"

#include "constants.h"
#include "units.h"

#include <cmath>

namespace {
    constexpr double Atm = 101325.0;
    const GasSystem::Properties *Real = &GasSystem::Properties::Real;

    GasSystem::Mix air() {
        GasSystem::Mix mix;
        mix.p_inert = 0.75;
        mix.p_o2 = 0.25;
        return mix;
    }

    GasSystem::Mix burned() {
        GasSystem::Mix mix;
        mix.p_inert = 1.0;
        mix.p_o2 = 0.0;
        return mix;
    }

    // Fresh charge as the intake mixes it: about 2.4% fuel vapour by moles
    GasSystem::Mix fresh() {
        GasSystem::Mix mix;
        mix.p_fuel = 0.024;
        mix.p_inert = 0.976 * 0.75;
        mix.p_o2 = 0.976 * 0.25;
        return mix;
    }

    GasSystem make(double P, double V, double T, const GasSystem::Mix &mix) {
        GasSystem g;
        g.setProperties(Real);
        g.initialize(P, V, T, mix);
        return g;
    }
}

// The heat capacity ratios engines see: cold air, fresh charge, and burned gas when hot
TEST(RealGasTests, HeatCapacityRatios) {
    EXPECT_NEAR(make(Atm, 1e-3, 300, air()).heatCapacityRatio(), 1.39, 0.01);
    EXPECT_NEAR(make(Atm, 1e-3, 300, fresh()).heatCapacityRatio(), 1.34, 0.01);
    EXPECT_NEAR(make(Atm, 1e-3, 2000, burned()).heatCapacityRatio(), 1.25, 0.01);
    // Upstream's gas stays at 1.4
    GasSystem ideal;
    ideal.initialize(Atm, 1e-3, 2000, burned());
    EXPECT_DOUBLE_EQ(ideal.heatCapacityRatio(), 1.4);
}

TEST(RealGasTests, StateRoundTrips) {
    const GasSystem g = make(3 * Atm, 2e-4, 1500, burned());
    EXPECT_NEAR(g.temperature(), 1500, 1e-6);
    EXPECT_NEAR(g.pressure(), 3 * Atm, 1e-3);

    // Switching model keeps pressure and temperature
    GasSystem h;
    h.initialize(2 * Atm, 1e-3, 900, air());
    h.setProperties(Real);
    EXPECT_NEAR(h.temperature(), 900, 1e-6);
    EXPECT_NEAR(h.pressure(), 2 * Atm, 1e-3);
}

// A slow 10:1 compression of air follows the real isentrope, a ln(T2/T1) + b (T2 - T1) = R ln 10,
// and ends cooler than with a constant gamma of 1.4
TEST(RealGasTests, AdiabaticCompression) {
    GasSystem g = make(Atm, 1e-3, 300, air());
    const int steps = 20000;
    const double V1 = 1e-3, V2 = 1e-4;
    for (int i = 0; i < steps; ++i) g.changeVolume((V2 - V1) / steps);

    const GasSystem::HeatCapacity h = GasSystem::heatCapacity(*Real, air());
    double T2 = 700;
    for (int i = 0; i < 50; ++i) {
        const double f = h.a * std::log(T2 / 300) + h.b * (T2 - 300) - constants::R * std::log(10.0);
        T2 -= f / (h.a / T2 + h.b);
    }
    EXPECT_NEAR(g.temperature() / T2, 1.0, 0.005);
    EXPECT_LT(g.temperature(), 300 * std::pow(10.0, 0.4) * 0.97);
}

// Burning at room temperature without adding the heat leaves it at room temperature: the
// reaction settles up the energy products and reactants hold there
TEST(RealGasTests, ReactionAtReferenceTemperature) {
    GasSystem g = make(Atm, 1e-3, units::celcius(25.0), fresh());
    g.react(g.n(), g.mix());
    EXPECT_NEAR(g.temperature(), units::celcius(25.0), 1e-6);
}

// Hot exhaust and cold fresh charge flowing back and forth: nothing made or lost
TEST(RealGasTests, FlowConservesMolesAndEnergy) {
    GasSystem a = make(4 * Atm, 5e-4, 1800, burned());
    GasSystem b = make(Atm, 1e-3, 320, fresh());
    a.setGeometry(0.1, 0.05, 1, 0);
    b.setGeometry(0.1, 0.05, 1, 0);

    const double n0 = a.n() + b.n();
    const double e0 = a.totalEnergy() + b.totalEnergy();
    GasSystem::FlowParameters params;
    params.k_flow = GasSystem::k_carb(100);
    params.dt = 1e-5;
    params.direction_x = 1;
    params.direction_y = 0;
    params.crossSectionArea_0 = params.crossSectionArea_1 = 1e-3;
    params.system_0 = &a;
    params.system_1 = &b;
    for (int i = 0; i < 2000; ++i) GasSystem::flow(params);

    EXPECT_NEAR((a.n() + b.n()) / n0, 1.0, 1e-9);
    EXPECT_NEAR((a.totalEnergy() + b.totalEnergy()) / e0, 1.0, 1e-6);
    EXPECT_NEAR(a.pressure() / b.pressure(), 1.0, 0.02);
}

// A pipe on the real model: sealed it conserves, and it trades gas with a volume exactly
TEST(RealGasTests, PipeConserves) {
    Pipe pipe;
    Pipe::Parameters params;
    params.length = 0.4;
    params.cells = 40;
    params.areas = { 1e-3 };
    params.pressure = Atm;
    params.temperature = 300;
    params.mix = fresh();
    pipe.initialize(params);
    pipe.setProperties(Real);
    EXPECT_NEAR(pipe.cellTemperature(5), 300, 1e-6);
    EXPECT_NEAR(pipe.cellPressure(5), Atm, 1e-3);

    for (int i = 0; i < 20; ++i) pipe.setCell(i, Pipe::makeCell(3 * Atm, 1200, 0, burned(), Real));
    GasSystem tank = make(2 * Atm, 0.002, 900, burned());
    const double mass0 = pipe.totalMass() + tank.mass();
    const double energy0 = pipe.totalEnergy() + tank.kineticEnergy();
    pipe.connect(1, &tank);
    pipe.setOrifice(1, GasSystem::k_carb(200));
    for (int k = 0; k < 20000; ++k) pipe.step(5e-6);
    EXPECT_NEAR((pipe.totalMass() + tank.mass()) / mass0, 1.0, 1e-9);
    EXPECT_NEAR((pipe.totalEnergy() + tank.kineticEnergy()) / energy0, 1.0, 1e-6);
}
