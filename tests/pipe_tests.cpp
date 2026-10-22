#include <gtest/gtest.h>

#include "pipe.h"

#include "constants.h"
#include "units.h"

#include <cmath>

namespace {
    constexpr double Atm = 101325.0;
    constexpr double Ambient = 298.15;

    Pipe makePipe(double length, int cells, double area = 1e-3, double friction = 0.0) {
        Pipe pipe;
        Pipe::Parameters params;
        params.length = length;
        params.cells = cells;
        params.areas = { area };
        params.pressure = Atm;
        params.temperature = Ambient;
        params.frictionFactor = friction;
        pipe.initialize(params);
        return pipe;
    }

    double soundSpeed(double T) {
        return std::sqrt(Pipe::Gamma * constants::R / units::AirMolecularMass * T);
    }

    // Time at which the pressure at cell i first rises (or falls) past a threshold
    double arrival(Pipe &pipe, int i, double threshold, double dt, double maxTime, bool rising = true) {
        for (double t = 0; t < maxTime; t += dt) {
            pipe.step(dt);
            const double p = pipe.cellPressure(i);
            if (rising ? p > threshold : p < threshold) return t + dt;
        }
        return -1;
    }
}

// A sealed pipe keeps its mass and energy however violently the gas sloshes about
TEST(PipeTests, ClosedPipeConserves) {
    Pipe pipe = makePipe(0.5, 50);
    for (int i = 0; i < 25; ++i) pipe.setCell(i, Pipe::makeCell(3 * Atm, 600, 0));
    const double m0 = pipe.totalMass(), e0 = pipe.totalEnergy();
    for (int k = 0; k < 2000; ++k) pipe.step(5e-6);
    EXPECT_NEAR(pipe.totalMass() / m0, 1.0, 1e-9);
    EXPECT_NEAR(pipe.totalEnergy() / e0, 1.0, 1e-9);
}

// Sod's shock tube: pressure 10:1 and density 8:1 across a diaphragm at the middle. The exact
// solution (gamma 1.4) has the shock at x = 0.5 + 1.75216 t, the contact at 0.5 + 0.92745 t
// and a plateau pressure of 0.30313 of the high side, in units where the high side's
// pressure and density are 1 (velocities scale by sqrt(p/rho)).
TEST(PipeTests, SodShockTube) {
    const int n = 800;
    Pipe pipe = makePipe(1.0, n);
    const double pL = Atm, rhoL = 1.0, pR = 0.1 * Atm, rhoR = 0.125;
    const double R = constants::R / units::AirMolecularMass;
    for (int i = 0; i < n; ++i) {
        const bool left = i < n / 2;
        pipe.setCell(i, Pipe::makeCell(left ? pL : pR, (left ? pL / rhoL : pR / rhoR) / R, 0));
    }

    const double scale = std::sqrt(pL / rhoL);   // m/s per unit velocity
    const double tNondim = 0.2;
    const double t = tNondim / scale;            // the pipe is 1 m long
    const int steps = 400;
    for (int k = 0; k < steps; ++k) pipe.step(t / steps);

    // Shock: the last place the pressure is clearly above the low side
    int shock = 0;
    for (int i = n / 2; i < n; ++i) if (pipe.cellPressure(i) > 0.5 * (pR + 0.30313 * pL)) shock = i;
    const double shockX = (shock + 0.5) / n;
    EXPECT_NEAR(shockX, 0.5 + 1.75216 * tNondim, 0.02);

    // Plateau between the rarefaction and the shock
    const int plateau = static_cast<int>((0.5 + 0.8 * tNondim * 1.0) * n);
    EXPECT_NEAR(pipe.cellPressure(plateau) / pL, 0.30313, 0.02);
}

// A small pressure pulse travels at the speed of sound
TEST(PipeTests, WavesTravelAtTheSpeedOfSound) {
    const double length = 2.0;
    const int n = 400;
    Pipe pipe = makePipe(length, n);
    for (int i = 0; i < 4; ++i) pipe.setCell(i, Pipe::makeCell(1.05 * Atm, Ambient * std::pow(1.05, 0.4 / 1.4), 0));

    const double expected = (length - 4.0 * length / n) / soundSpeed(Ambient);
    const double t = arrival(pipe, n - 1, 1.01 * Atm, 2e-6, 0.02);
    EXPECT_NEAR(t, expected, 0.05 * expected);
}

// A pressure pulse comes back from a closed end as a pressure pulse, and from an open end
// (a huge volume at ambient) as a suction pulse: what tuned runners and headers rely on
TEST(PipeTests, ReflectionsFromClosedAndOpenEnds) {
    for (const bool open : { false, true }) {
        const double length = 1.0;
        const int n = 200;
        Pipe pipe = makePipe(length, n);
        for (int i = 0; i < 6; ++i) pipe.setCell(i, Pipe::makeCell(1.1 * Atm, Ambient * std::pow(1.1, 0.4 / 1.4), 0));

        GasSystem room;
        room.initialize(Atm, 1000.0, Ambient);

        if (open) {
            pipe.connect(1, &room);
            pipe.setOrifice(1, GasSystem::k_carb(50000)); // Far wider than the pipe: an open end
        }

        // Watch the start of the pipe once the pulse has been to the far end and back
        const double dt = 2e-6;
        const double back = 2 * length / soundSpeed(Ambient);
        double extreme = Atm;
        for (double t = 0; t < 1.3 * back; t += dt) {
            pipe.step(dt);
            if (t > 0.8 * back) {
                const double p = pipe.cellPressure(2);
                if (std::abs(p - Atm) > std::abs(extreme - Atm)) extreme = p;
            }
        }
        // The pulse starts at +10%, about +5% once it's spread into a travelling wave, and the
        // reflection doubles again against the closed start of the pipe
        if (open) EXPECT_LT(extreme, 0.95 * Atm) << "an open end sends back strong suction";
        else EXPECT_GT(extreme, 1.05 * Atm) << "a closed end sends back strong pressure";
    }
}

// Gas exchanged with a volume is conserved: what one gains the other loses
TEST(PipeTests, ExchangeWithAVolumeConserves) {
    Pipe pipe = makePipe(0.3, 30);
    GasSystem tank;
    tank.initialize(2 * Atm, 0.002, 400);

    const double M = units::AirMolecularMass;
    const double mass0 = pipe.totalMass() + tank.n() * M;
    const double energy0 = pipe.totalEnergy() + tank.kineticEnergy();
    pipe.connect(0, &tank);
    pipe.setOrifice(0, GasSystem::k_carb(200));
    for (int k = 0; k < 20000; ++k) pipe.step(5e-6);
    EXPECT_NEAR((pipe.totalMass() + tank.n() * M) / mass0, 1.0, 1e-9);
    EXPECT_NEAR((pipe.totalEnergy() + tank.kineticEnergy()) / energy0, 1.0, 1e-6);
    // And it settles: the pressures even out
    EXPECT_NEAR(pipe.cellPressure(0) / tank.pressure(), 1.0, 0.02);
}
