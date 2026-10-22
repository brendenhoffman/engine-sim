#ifndef ATG_ENGINE_SIM_PIPE_H
#define ATG_ENGINE_SIM_PIPE_H

#include "gas_system.h"

#include <vector>

// A one-dimensional pipe: the gas in it is split into cells along its length and moves by the
// Euler equations (mass, momentum, energy), so pressure waves travel, reflect off the ends and
// arrive when they should. That gives intake ram tuning, exhaust scavenging and reversion from
// the physics rather than from lumped volumes. The area may change along the pipe (bellmouths,
// tapers, expansion chambers).
//
// The ends connect to ordinary 0-D volumes (a plenum, a cylinder, a collector) through an
// orifice given as an engine-sim flow constant. The flow through an end is solved like the
// flow between two cells, between the end of the pipe and the volume's gas at rest, and
// weighted by how much of the pipe's area the orifice opens (engine-sim's flow constants are
// the standard orifice law, so they have an effective area). A fully open end reflects waves as
// an open end does, a shut one as a wall, and whatever leaves one side enters the other.
//
// Gas properties match GasSystem: one molar mass for every species, and the same heat capacities.
class Pipe {
    public:
        // Conserved quantities per unit volume
        struct Cell {
            double rho = 0;      // kg/m^3
            double momentum = 0; // kg/(m^2 s), along the pipe
            double energy = 0;   // J/m^3, internal plus kinetic
            double rhoFuel = 0;  // kg/m^3 of fuel and of oxygen; the rest is inert
            double rhoO2 = 0;
        };

        // Primitive state: density, velocity, pressure, fuel and oxygen fractions
        struct Prim { double rho, u, p, yf, yo; };
        // Flux of each conserved quantity through a face, per unit area
        struct Flux { double mass, momentum, energy, fuel, o2; };

        struct Parameters {
            double length = 0.3;
            std::vector<double> areas;  // Per cell; a single value means constant area
            int cells = 10;
            double pressure = 101325;
            double temperature = 298.15;
            GasSystem::Mix mix;
            double frictionFactor = 0.02; // Darcy
        };

        void initialize(const Parameters &params);
        bool initialized() const { return !m_cells.empty(); }

        // Moves the gas inside the pipe on by dt (sub-stepping as stability needs; short slices
        // are saved up until there's enough for a stable step)
        void step(double dt);

        // Connects an end (0: start, 1: far end) to a volume, through an orifice whose flow
        // constant is set (every step, for valves) with setOrifice; 0 shuts it. An end can have
        // several (a rotor housing's port, which each face passes in turn); returns which.
        int connect(int end, GasSystem *volume);
        void setOrifice(int end, double k, int connection = 0) { m_ends[end].connections[connection].k = k; }
        // Moles that went into the pipe through a connection since last asked (negative: out)
        double takeFlow(int end, int connection = 0);

        // The effective area of an engine-sim flow constant (orifice law: n' = k p0 psi / sqrt(R T0))
        static double orificeArea(double k);

        int cellCount() const { return static_cast<int>(m_cells.size()); }
        double length() const { return m_length; }
        double cellPressure(int i) const;
        double cellTemperature(int i) const;
        double cellVelocity(int i) const;
        double cellDensity(int i) const { return m_cells[i].rho; }
        GasSystem::Mix cellMix(int i) const;
        double area(int i) const { return m_area[i]; }

        // Direct access, for tests
        const Cell &cell(int i) const { return m_cells[i]; }
        void setCell(int i, const Cell &cell) { m_cells[i] = cell; }
        static Cell makeCell(double pressure, double temperature, double velocity, const GasSystem::Mix &mix = {},
            const GasSystem::Properties *properties = &GasSystem::Properties::Ideal);

        // Switches the gas model (see GasSystem::Properties), keeping pressures and temperatures
        void setProperties(const GasSystem::Properties *properties);

        // Totals, for tests and diagnostics
        double totalMass() const;
        double totalEnergy() const;

        static constexpr int DegreesOfFreedom = 5;
        static constexpr double Gamma = 1.0 + 2.0 / DegreesOfFreedom;

    private:
        double pressure(const Cell &c) const;
        double soundSpeed(const Cell &c) const;
        double temperature(const Cell &c) const;
        const GasSystem::Properties *m_properties = &GasSystem::Properties::Ideal;
        void substep(double dt);
        void finishCell(Cell &out, size_t i, double dt);

        std::vector<Cell> m_cells;
        std::vector<Prim> m_prim, m_slopes, m_faceL, m_faceR; // Scratch for each step
        std::vector<GasSystem::HeatCapacity> m_heat;          // Each cell's, for this step
        GasSystem::HeatCapacity cellHeat(const Cell &c) const;
        std::vector<double> m_area;      // Cell areas
        std::vector<double> m_faceArea;  // Between cells (m_cells.size() + 1, ends included)
        std::vector<double> m_friction;  // Darcy factor / (2 x hydraulic diameter), per cell
        std::vector<double> m_flux[5];   // Scratch: fluxes through each face
        double m_length = 0;
        double m_dx = 0;
        double m_frictionFactor = 0.02;
        double m_pending = 0;   // Time not yet stepped (see step())

        struct Connection {
            GasSystem *volume = nullptr;
            double k = 0;
            double moles = 0; // Into the pipe since takeFlow()
        };
        struct End {
            std::vector<Connection> connections;
        };
        End m_ends[2];
        void endFlux(int end, const Prim &pipeSide, const GasSystem::HeatCapacity &heat, double dt, double *flux);
        void connectionFlux(int end, const Connection &c, const Prim &pipeSide, const GasSystem::HeatCapacity &heat,
            double area, double dt, double *open, Flux *through) const;
        std::vector<double> m_scratchOpen;
        std::vector<Flux> m_scratchThrough;
        double m_maxSpeed = 0;  // Fastest wave as of the last step
};

#endif /* ATG_ENGINE_SIM_PIPE_H */
