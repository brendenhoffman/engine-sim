#include "pipe.h"

#include "constants.h"
#include "units.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
    constexpr double MolarMass = units::AirMolecularMass;          // kg/mol, as GasSystem
    constexpr double SpecificR = constants::R / units::AirMolecularMass;
    constexpr double MinDensity = 1e-6;
    const double IdealChokedLimit = std::pow(2.0 / 2.4, 1.4 / 0.4);
    const double IdealChokedRate = std::sqrt(1.4) * std::pow(2 / 2.4, 2.4 / 0.8);
    constexpr double MinPressure = 1.0;

    using Flux = Pipe::Flux;
    using Prim = Pipe::Prim;
    using Properties = GasSystem::Properties;
    using HeatCapacity = GasSystem::HeatCapacity;

    HeatCapacity heat(const Properties *properties, double yf, double yo) {
        if (properties == &Properties::Ideal) return HeatCapacity();
        GasSystem::Mix mix;
        mix.p_fuel = yf;
        mix.p_o2 = yo;
        mix.p_inert = 1 - yf - yo;
        return GasSystem::heatCapacity(*properties, mix);
    }

    // Internal energy per unit volume of gas at a pressure and density
    double internalEnergy(const HeatCapacity &h, double rho, double p) {
        if (h.b == 0) return p * h.a / constants::R;
        return rho / MolarMass * h.energy(p / (rho * SpecificR));
    }

    // Heat capacity ratio of a primitive state
    double gammaOf(const HeatCapacity &h, const Prim &w) {
        return h.gamma(w.p / (w.rho * SpecificR));
    }

    Flux physicalFlux(const Prim &w, double E) {
        return { w.rho * w.u, w.rho * w.u * w.u + w.p, (E + w.p) * w.u, w.rho * w.u * w.yf, w.rho * w.u * w.yo };
    }

    // HLLC flux (Toro): keeps contact surfaces, such as the edge between fresh charge and
    // exhaust pushed back into a runner, sharp. Its star states hold for any equation of state;
    // only the energies and wave speeds need the gas's heat capacity.
    Flux hllc(const Prim &L, const Prim &R, const HeatCapacity &hL, const HeatCapacity &hR) {
        const double cL = std::sqrt(gammaOf(hL, L) * L.p / L.rho), cR = std::sqrt(gammaOf(hR, R) * R.p / R.rho);
        const double EL = internalEnergy(hL, L.rho, L.p) + 0.5 * L.rho * L.u * L.u;
        const double ER = internalEnergy(hR, R.rho, R.p) + 0.5 * R.rho * R.u * R.u;
        const double SL = std::min(L.u - cL, R.u - cR);
        const double SR = std::max(L.u + cL, R.u + cR);
        if (SL >= 0) return physicalFlux(L, EL);
        if (SR <= 0) return physicalFlux(R, ER);

        const double denominator = L.rho * (SL - L.u) - R.rho * (SR - R.u);
        const double Sstar = (R.p - L.p + L.rho * L.u * (SL - L.u) - R.rho * R.u * (SR - R.u)) / denominator;

        auto star = [&](const Prim &K, double E, double S) {
            const double factor = K.rho * (S - K.u) / (S - Sstar);
            const double eStar = E / K.rho + (Sstar - K.u) * (Sstar + K.p / (K.rho * (S - K.u)));
            return Flux{ factor, factor * Sstar, factor * eStar, factor * K.yf, factor * K.yo };
        };
        const Prim &K = (Sstar >= 0) ? L : R;
        const double EK = (Sstar >= 0) ? EL : ER;
        const double SK = (Sstar >= 0) ? SL : SR;
        const Flux F = physicalFlux(K, EK);
        const Flux U = star(K, EK, SK);
        const double rhoK = K.rho, mK = K.rho * K.u;
        return { F.mass + SK * (U.mass - rhoK), F.momentum + SK * (U.momentum - mK), F.energy + SK * (U.energy - EK),
                 F.fuel + SK * (U.fuel - rhoK * K.yf), F.o2 + SK * (U.o2 - rhoK * K.yo) };
    }

    // van Leer limited slope
    double slope(double a, double b) {
        return (a * b > 0) ? 2 * a * b / (a + b) : 0.0;
    }
}

void Pipe::initialize(const Parameters &params) {
    const int n = std::max(params.cells, 2);
    m_length = params.length;
    m_dx = m_length / n;
    m_frictionFactor = params.frictionFactor;

    m_area.assign(n, params.areas.empty() ? 1e-3 : params.areas[0]);
    if (params.areas.size() == static_cast<size_t>(n)) m_area = params.areas;
    m_faceArea.assign(n + 1, 0);
    m_faceArea[0] = m_area[0];
    m_faceArea[n] = m_area[n - 1];
    for (int i = 1; i < n; ++i) m_faceArea[i] = 0.5 * (m_area[i - 1] + m_area[i]);

    m_cells.assign(n, makeCell(params.pressure, params.temperature, 0, params.mix, m_properties));
    for (auto &f : m_flux) f.assign(n + 1, 0);
    m_friction.resize(n);
    for (int i = 0; i < n; ++i) m_friction[i] = m_frictionFactor / (2 * std::sqrt(4 * m_area[i] / constants::pi));
}

Pipe::Cell Pipe::makeCell(double pressure, double temperature, double velocity, const GasSystem::Mix &mix,
    const GasSystem::Properties *properties)
{
    Cell c;
    c.rho = pressure / (SpecificR * temperature);
    c.momentum = c.rho * velocity;
    c.energy = internalEnergy(heat(properties, mix.p_fuel, mix.p_o2), c.rho, pressure) + 0.5 * c.rho * velocity * velocity;
    c.rhoFuel = c.rho * mix.p_fuel;
    c.rhoO2 = c.rho * mix.p_o2;
    return c;
}

void Pipe::setProperties(const GasSystem::Properties *properties) {
    for (size_t i = 0; i < m_cells.size(); ++i) {
        const Cell &c = m_cells[i];
        m_cells[i] = makeCell(pressure(c), temperature(c), c.momentum / c.rho, cellMix(static_cast<int>(i)), properties);
    }
    m_properties = properties;
    m_maxSpeed = 0;
}

GasSystem::HeatCapacity Pipe::cellHeat(const Cell &c) const {
    return heat(m_properties, c.rhoFuel / c.rho, c.rhoO2 / c.rho);
}

double Pipe::temperature(const Cell &c) const {
    const double kinetic = 0.5 * c.momentum * c.momentum / c.rho;
    const HeatCapacity h = cellHeat(c);
    const double perMol = std::max(c.energy - kinetic, 0.0) * MolarMass / c.rho;
    return std::max(h.temperature(perMol), MinPressure / (c.rho * SpecificR));
}

double Pipe::pressure(const Cell &c) const {
    return c.rho * SpecificR * temperature(c);
}

double Pipe::soundSpeed(const Cell &c) const {
    const double T = temperature(c);
    return std::sqrt(heat(m_properties, c.rhoFuel / c.rho, c.rhoO2 / c.rho).gamma(T) * SpecificR * T);
}

double Pipe::cellPressure(int i) const { return pressure(m_cells[i]); }
double Pipe::cellTemperature(int i) const { return temperature(m_cells[i]); }
double Pipe::cellVelocity(int i) const { return m_cells[i].momentum / m_cells[i].rho; }

GasSystem::Mix Pipe::cellMix(int i) const {
    const Cell &c = m_cells[i];
    GasSystem::Mix mix;
    mix.p_fuel = std::clamp(c.rhoFuel / c.rho, 0.0, 1.0);
    mix.p_o2 = std::clamp(c.rhoO2 / c.rho, 0.0, 1.0 - mix.p_fuel);
    mix.p_inert = 1.0 - mix.p_fuel - mix.p_o2;
    return mix;
}

double Pipe::totalMass() const {
    double m = 0;
    for (size_t i = 0; i < m_cells.size(); ++i) m += m_cells[i].rho * m_area[i] * m_dx;
    return m;
}

double Pipe::totalEnergy() const {
    double e = 0;
    for (size_t i = 0; i < m_cells.size(); ++i) e += m_cells[i].energy * m_area[i] * m_dx;
    return e;
}

void Pipe::step(double dt) {
    if (m_cells.empty() || dt <= 0) return;

    // Advance only once enough time has built up for a reasonably full stable step: callers
    // step in small slices (the fluid substeps) that are far shorter than a wave takes to
    // cross a cell. The ends still exchange gas every slice. Triggering at a Courant number
    // of 0.6 leaves room for the last slice that pushes it over.
    m_pending += dt;
    if (m_maxSpeed <= 0) {
        for (const Cell &c : m_cells) m_maxSpeed = std::max(m_maxSpeed, std::abs(c.momentum / c.rho) + soundSpeed(c));
    }
    // An end that has just opened onto hot gas (a cylinder at blowdown) brings waves the last
    // step never saw: up to sonic flow plus its sound speed
    double speed = m_maxSpeed;
    for (const End &e : m_ends) {
        for (const Connection &c : e.connections) {
            if (c.volume != nullptr && c.k > 0 && c.volume->n() > 0 && c.volume->temperature() > 0) {
                speed = std::max(speed, 2 * std::sqrt(c.volume->heatCapacityRatio() * SpecificR * c.volume->temperature()));
            }
        }
    }
    if (m_pending * speed < 0.6 * m_dx) return;

    const int substeps = std::max(1, static_cast<int>(std::ceil(m_pending * speed / (0.85 * m_dx))));
    const double h = m_pending / substeps;
    m_pending = 0;
    for (int i = 0; i < substeps; ++i) substep(h);
}

// MUSCL-Hancock: limited linear reconstruction (van Leer) of the primitive variables in each
// cell, a half-step predictor of the face values, then one HLLC flux evaluation per face.
// Second order in space and time at the cost of a single flux sweep.
void Pipe::substep(double dt) {
    const int n = static_cast<int>(m_cells.size());
    const double halfStep = 0.5 * dt / m_dx;
    m_prim.resize(n);
    m_slopes.resize(n);
    m_faceL.resize(n);
    m_faceR.resize(n);
    m_heat.resize(n);

    double maxSpeed = 1;
    for (int i = 0; i < n; ++i) {
        const Cell &c = m_cells[i];
        const HeatCapacity h = cellHeat(c);
        const double u = c.momentum / c.rho;
        const double perMol = std::max(c.energy - 0.5 * c.momentum * u, 0.0) * MolarMass / c.rho;
        const double T = std::max(h.temperature(perMol), MinPressure / (c.rho * SpecificR));
        m_heat[i] = h;
        m_prim[i] = { c.rho, u, c.rho * SpecificR * T, c.rhoFuel / c.rho, c.rhoO2 / c.rho };
        maxSpeed = std::max(maxSpeed, std::abs(u) + std::sqrt(h.gamma(T) * SpecificR * T));
    }
    for (const End &e : m_ends) {
        for (const Connection &c : e.connections) {
            if (c.volume != nullptr && c.k > 0 && c.volume->n() > 0) {
                maxSpeed = std::max(maxSpeed, 2 * std::sqrt(c.volume->heatCapacityRatio() * SpecificR * c.volume->temperature()));
            }
        }
    }
    m_maxSpeed = maxSpeed;
    for (int i = 0; i < n; ++i) {
        const Prim &b = m_prim[i];
        // Beyond the ends: a mirror (reflecting wall) with the velocity reversed
        Prim a = m_prim[i > 0 ? i - 1 : 0], c = m_prim[i < n - 1 ? i + 1 : n - 1];
        if (i == 0) a.u = -a.u;
        if (i == n - 1) c.u = -c.u;
        Prim d{ slope(b.rho - a.rho, c.rho - b.rho), slope(b.u - a.u, c.u - b.u), slope(b.p - a.p, c.p - b.p),
                slope(b.yf - a.yf, c.yf - b.yf), slope(b.yo - a.yo, c.yo - b.yo) };
        Prim L{ b.rho - 0.5 * d.rho, b.u - 0.5 * d.u, b.p - 0.5 * d.p, b.yf - 0.5 * d.yf, b.yo - 0.5 * d.yo };
        Prim R{ b.rho + 0.5 * d.rho, b.u + 0.5 * d.u, b.p + 0.5 * d.p, b.yf + 0.5 * d.yf, b.yo + 0.5 * d.yo };
        if (L.rho <= 0 || L.p <= 0 || R.rho <= 0 || R.p <= 0) {
            L = R = b; // First order where a slope would go unphysical
        }
        else {
            // Half-step evolution of both face values by the cell's own flux difference
            // (primitive form: rho, u, p, fractions)
            const double g = gammaOf(m_heat[i], b);
            const double drho = -halfStep * (R.rho * R.u - L.rho * L.u);
            const double du = -halfStep * ((R.u * R.u - L.u * L.u) * 0.5 + (R.p - L.p) / b.rho);
            const double dp = -halfStep * (b.u * (R.p - L.p) + g * b.p * (R.u - L.u));
            const double dyf = -halfStep * b.u * (R.yf - L.yf);
            const double dyo = -halfStep * b.u * (R.yo - L.yo);
            L.rho += drho; L.u += du; L.p += dp; L.yf += dyf; L.yo += dyo;
            R.rho += drho; R.u += du; R.p += dp; R.yf += dyf; R.yo += dyo;
            if (L.rho <= 0 || L.p <= 0 || R.rho <= 0 || R.p <= 0) L = R = b;
        }
        m_faceL[i] = L;
        m_faceR[i] = R;
    }

    for (int f = 0; f <= n; ++f) {
        double flux[5];
        if (f == 0) endFlux(0, m_faceL[0], m_heat[0], dt, flux);
        else if (f == n) endFlux(1, m_faceR[n - 1], m_heat[n - 1], dt, flux);
        else {
            const Flux inner = hllc(m_faceR[f - 1], m_faceL[f], m_heat[f - 1], m_heat[f]);
            flux[0] = inner.mass; flux[1] = inner.momentum; flux[2] = inner.energy; flux[3] = inner.fuel; flux[4] = inner.o2;
        }
        const double a = m_faceArea[f];
        for (int q = 0; q < 5; ++q) m_flux[q][f] = flux[q] * a;
    }

    for (int i = 0; i < n; ++i) {
        Cell &c = m_cells[i];
        const double k = dt / (m_area[i] * m_dx);
        c.rho -= k * (m_flux[0][i + 1] - m_flux[0][i]);
        c.momentum += -k * (m_flux[1][i + 1] - m_flux[1][i])
            + k * m_prim[i].p * (m_faceArea[i + 1] - m_faceArea[i]); // Walls of a changing area push back
        c.energy -= k * (m_flux[2][i + 1] - m_flux[2][i]);
        c.rhoFuel -= k * (m_flux[3][i + 1] - m_flux[3][i]);
        c.rhoO2 -= k * (m_flux[4][i + 1] - m_flux[4][i]);
        finishCell(c, i, dt);
    }
}

void Pipe::finishCell(Cell &out, size_t i, double dt) {
    // Keep the state physical
    out.rho = std::max(out.rho, MinDensity);
    out.rhoFuel = std::clamp(out.rhoFuel, 0.0, out.rho);
    out.rhoO2 = std::clamp(out.rhoO2, 0.0, out.rho - out.rhoFuel);

    // Wall friction (implicit, so it can only slow the flow); its work becomes heat
    const double u = out.momentum / out.rho;
    out.momentum /= 1 + dt * m_friction[i] * std::abs(u);

    const double kinetic = 0.5 * out.momentum * out.momentum / out.rho;
    out.energy = std::max(out.energy, kinetic + 2.5 * MinPressure);
}

int Pipe::connect(int end, GasSystem *volume) {
    Connection c;
    c.volume = volume;
    m_ends[end].connections.push_back(c);
    return static_cast<int>(m_ends[end].connections.size()) - 1;
}

double Pipe::takeFlow(int end, int connection) {
    Connection &c = m_ends[end].connections[connection];
    const double moles = c.moles;
    c.moles = 0;
    return moles;
}

double Pipe::orificeArea(double k) {
    // engine-sim: n' = k p0 psi / sqrt(R T0) moles/s; the orifice law: m' = A p0 psi / sqrt(R_s T0)
    // kg/s, so n' = A p0 psi / (sqrt(M) sqrt(R T0)) and A = k sqrt(M)
    return k * std::sqrt(MolarMass);
}

// Flux through an end face (per unit area, positive along x) and its exchange with the volume
void Pipe::endFlux(int end, const Prim &pipeSide, const HeatCapacity &pipeHeat, double dt, double *out) {
    const int n = static_cast<int>(m_cells.size());
    const int face = (end == 0) ? 0 : n;
    const double area = m_faceArea[face];

    Prim mirror = pipeSide;
    mirror.u = -mirror.u;
    const Flux wall = (end == 0) ? hllc(mirror, pipeSide, pipeHeat, pipeHeat) : hllc(pipeSide, mirror, pipeHeat, pipeHeat);

    // Each open connection passes its share; the rest of the end acts as a wall
    m_scratchOpen.clear();
    m_scratchThrough.clear();
    double totalOpen = 0;
    for (Connection &c : m_ends[end].connections) {
        double open = 0;
        Flux through{ 0, 0, 0, 0, 0 };
        connectionFlux(end, c, pipeSide, pipeHeat, area, dt, &open, &through);
        m_scratchOpen.push_back(open);
        m_scratchThrough.push_back(through);
        totalOpen += open;
    }
    const double scale = totalOpen > 1 ? 1 / totalOpen : 1.0;

    for (int q = 0; q < 5; ++q) out[q] = (1 - totalOpen * scale) * (&wall.mass)[q];
    for (size_t j = 0; j < m_scratchOpen.size(); ++j) {
        const double open = m_scratchOpen[j] * scale;
        if (open <= 0) continue;
        const Flux &through = m_scratchThrough[j];
        for (int q = 0; q < 5; ++q) out[q] += open * (&through.mass)[q];

        // The volume gets the other side of what went through
        Connection &c = m_ends[end].connections[j];
        const double sign = (end == 0) ? 1.0 : -1.0; // + : into the pipe
        const double mass = sign * through.mass * open * area * dt;
        const double energy = sign * through.energy * open * area * dt;
        const double moles = mass / MolarMass;
        if (moles > 0) {
            if (c.volume->hasOutflowMix()) c.volume->loseN(moles, energy / moles, c.volume->outflowMix());
            else c.volume->loseN(moles, energy / moles);
        }
        else if (moles < 0) {
            GasSystem::Mix mix;
            const double m = std::abs(through.mass);
            mix.p_fuel = (m > 0) ? std::clamp(std::abs(through.fuel) / m, 0.0, 1.0) : 0.0;
            mix.p_o2 = (m > 0) ? std::clamp(std::abs(through.o2) / m, 0.0, 1.0 - mix.p_fuel) : 0.0;
            mix.p_inert = 1 - mix.p_fuel - mix.p_o2;
            c.volume->gainN(-moles, energy / moles, mix);
        }
        c.moles += moles;
    }
}

// How much one connection opens the end (0-1, the series combination of the pipe's own flow
// and the orifice's) and the flux through it if it were wide open
void Pipe::connectionFlux(int end, const Connection &c, const Prim &pipeSide, const HeatCapacity &pipeHeat,
    double area, double dt, double *open, Flux *through) const
{
    *open = 0;
    if (c.volume == nullptr || c.k <= 0 || c.volume->n() <= 0 || c.volume->temperature() <= 0) return;

    const GasSystem::Mix mix = c.volume->outflowMix();
    const double outward = (end == 0) ? -1.0 : 1.0;

    // A ghost cell beyond the end, held at the volume's pressure with the pipe's velocity
    // carried across: gas leaving is the pipe's own, expanded to that pressure; gas entering
    // is the volume's. Pinning the pressure there is what makes an open end send a pressure
    // pulse back as suction. The flux comes from the same Riemann solver as between cells.
    const double Pv = c.volume->pressure();
    const double Tv = c.volume->temperature();
    const bool leaving = pipeSide.u * outward > 0;
    Prim ghost;
    if (leaving) {
        const double g = gammaOf(pipeHeat, pipeSide);
        ghost = { pipeSide.rho * std::pow(Pv / pipeSide.p, 1 / g), pipeSide.u, Pv, pipeSide.yf, pipeSide.yo };
    }
    else {
        // Entering: the volume's gas starts at rest, at its stagnation state, and pays for its
        // speed with static pressure and temperature (giving it the volume's full pressure as
        // well as the speed was a free push that fed the ram effect)
        const double g = c.volume->heatCapacityRatio();
        const double Cp = g / (g - 1) * SpecificR;
        const double u = std::min(std::abs(pipeSide.u), std::sqrt(2 * Cp * Tv) * 0.95);
        const double Ts = Tv - u * u / (2 * Cp);
        const double ps = Pv * std::pow(Ts / Tv, g / (g - 1));
        ghost = { ps / (SpecificR * Ts), pipeSide.u, ps, mix.p_fuel, mix.p_o2 };
    }
    const HeatCapacity ghostHeat = leaving ? pipeHeat : heat(m_properties, mix.p_fuel, mix.p_o2);
    *through = (end == 0) ? hllc(ghost, pipeSide, ghostHeat, pipeHeat) : hllc(pipeSide, ghost, pipeHeat, ghostHeat);

    // That is the flow a fully open end would pass, limited by the pipe's own waves. The
    // orifice (a valve, a port) is a second restriction in series with it: for two flows
    // that go as the square root of their pressure drop, 1/m^2 = 1/m_pipe^2 + 1/m_orifice^2.
    const double pipeFlow = std::abs(through->mass) * area;
    if (pipeFlow <= 0) return;
    const double outwardSpeed = pipeSide.u * outward;
    const double pEnd = pipeSide.p + 0.5 * pipeSide.rho * outwardSpeed * std::abs(outwardSpeed); // Ram either way
    const double Tend = pipeSide.p / (pipeSide.rho * SpecificR);
    // Upstream gas's heat capacity ratio
    const double hcr = (pEnd > Pv) ? gammaOf(pipeHeat, pipeSide) : c.volume->heatCapacityRatio();
    double chokedLimit = IdealChokedLimit, chokedRate = IdealChokedRate;
    if (m_properties != &Properties::Ideal) GasSystem::chokedConstantsAt(hcr, &chokedLimit, &chokedRate);
    const double orificeFlow = MolarMass * std::abs((pEnd > Pv)
        ? GasSystem::flowRate(c.k, pEnd, Pv, Tend, Tv, hcr, chokedLimit, chokedRate)
        : GasSystem::flowRate(c.k, Pv, pEnd, Tv, Tend, hcr, chokedLimit, chokedRate));
    if (orificeFlow <= 0) return;
    *open = 1 / std::sqrt(1 / (pipeFlow * pipeFlow) + 1 / (orificeFlow * orificeFlow)) / pipeFlow;

    // Mass the volume would give this step; never more than half of what it holds
    const double intoPipe = ((end == 0) ? through->mass : -through->mass) * *open * area * dt;
    const double volumeMass = c.volume->n() * MolarMass;
    if (intoPipe > 0.5 * volumeMass) *open *= 0.5 * volumeMass / intoPipe;
}
