# engine-sim as a reference for dyno-sim

What this fork's models really are, where they're physical and where they're fitted or faked,
what each engine file had to be tuned with, and what that means for dyno-sim. Written
2026-10-23, after the diesel, forced induction, two-stroke and Detroit work.

dyno-sim's goal, for context: close to real, built from real-world figures turned into
functions. Plug in as many published specs as exist (bore, stroke, cam, turbo, pipes); where
they don't exist, describe the behaviour and let the model produce something that behaves
like the real part; or mix the two. Published power figures are imprecise anyway; it has to
add up, not match to the horsepower.

Scope:
- Only models still in use are covered. Upstream models this fork replaced are left out:
  ideal-gas properties (now real gas), the piston-wall friction (now FMEP), and the
  lumped-only gas exchange where engines now use the 1-D pipes.
- Upstream's shipped engines are mostly a **sound** reference. Their parts data is useful
  where noted (the GM LS head is the one most things were scaled from).

## How to read the status tags

| Tag | Meaning |
|---|---|
| **Physical** | Built from physics with real inputs; would carry over as is |
| **Correlation** | A published empirical correlation with its real constants |
| **Effective** | Right shape of behaviour, but its parameters aren't real-world specs: you can't look them up, only fit them |
| **Fitted** | A value tuned per engine until the result matched a published figure or a recording |
| **Sound hack** | Exists only to make the audio right; no physical counterpart |

## Model audit

### Gas and thermodynamics

| Model | Status | Notes |
|---|---|---|
| Gas properties (`real_gas`) | Physical | c_v linear in T per species (fuel, O2, inert). No dissociation; fine below ~2500 K. |
| Lumped gas volumes (GasSystem) | Physical | Cylinder, plenum, collectors, crankcase, charge volumes. |
| Orifice flow between volumes | Physical form, **effective** constants | Compressible orifice flow, but sized by an engine-sim flow constant (`k_carb(cfm)`), not an area and discharge coefficient. |
| Cylinder heat transfer | Effective | One coefficient, optionally scaled by mean piston speed^0.8. Not Woschni or Annand. 800 was used on the two-strokes, the default on the rest. |
| Friction (`friction_model 1`) | Correlation + **fitted** constant | The Chen-Flynn form (constant + peak pressure + piston speed terms). The constant was picked per engine: 15 kPa for two-strokes, 40-60 kPa for diesels. |

**dyno-sim:** use the orifice flow with area x Cd. Use Woschni (or Annand) heat transfer.
Chen-Flynn is fine, with its constant from engine type and bearing count.

### Breathing: valves, ports, runners, pipes

| Model | Status | Notes |
|---|---|---|
| Valve flow from flow-bench tables | Physical input type, **borrowed data** | CFM at 28 inH2O against lift is a real measurement, but few heads here have their own. The gas engines scale the GM LS table. The Series 60, NTC-444, C15 and C16 share one diesel table times `flow_attenuation` 1.6-2.0. The Detroits use a made-up four-valve table. |
| Cam lobes (harmonic profile) | Physical | Duration at 0.050 in, lift, centreline: real cam-card figures work. |
| Lumped intake and exhaust | Effective | 28 of 53 engines have no waves. Runner length and plenum volume barely matter there, which is why they never seemed to. |
| 1-D pipes (`pipe_resolution`) | Physical, with **two known problems** | MUSCL-HLLC, per-cell area (cones), real gas. **(1)** No wall friction or wall heat loss: two-stroke expansion chambers needed **1.3-1.6x their real lengths** to put the powerband in the right place. The likeliest cause is gas that stays too hot, so waves are too fast; never confirmed. **(2)** Short pipes get few cells (minimum 4), and the result depends on resolution: coarser cells sound brighter, so part of the "rasp" is numerical. |
| Pipe junctions | Missing | A pipe ends in a lumped collector; there are no branched junctions. Collector tuning (4-into-1, merge cones) isn't modelled. |

**dyno-sim:**
- **Valve flow:** compute it from valve diameter, lift and a Cd(L/D) curve, with seat angle and port area as options. Take a flow-bench table as an override when one exists.
- **Pipes:** 1-D everywhere it matters, with wall friction and wall heat transfer (that should remove the length fudge), real junctions, and a minimum cell size, not a minimum cell count.
- **Plenums:** keep them 0-D.

### Combustion

| Model | Status | Notes |
|---|---|---|
| Petrol flame (upstream) | Effective + **fitted** | Turbulent flame speed from a turbulence-to-speed function. Upstream's burns far too slowly (about 85 deg spark to 50% burned). 15 engines carry a multiplier: `fast_flame`/`two_stroke_flame` gain 3-10. |
| `max_burning_efficiency` | **Fitted** | Scales the energy released, so it scales torque directly. In 49 files; the project's engines used it to calibrate (0.67-0.70 on the turbo rotaries, 0.88 on the 2JZ, 0.95 on the diesels). |
| Residual-gas dilution, misfire | Effective | `dilution_sensitivity` slows the flame with burned gas. The `misfire_residual` threshold (0.65) makes two-stroke ring-ding. Real phenomena; the thresholds were set by ear. |
| Diesel ignition delay | Correlation | Hardenberg-Hase, integrated through compression, with the cetane number. |
| Diesel premixed share | Correlation | Watson's form. |
| Diesel burn shape | Effective | Double Wiebe with durations I set (premixed 6 + 4*rpm/2000 deg; diffusion 30 + 20*rpm/2000 + 20*phi deg). No injection rate, no spray, no wall wetting. |
| Diesel fuelling and governor | Effective + **fitted** | Fuel per injection, the smoke limit (phi) and a straight-line full-load curve (`peak_torque_speed` falling to `rated_fuel` at rated speed). All fitted to the published torque, not taken from injector or pump specs. |
| Shot scatter (`injector_spread`, `shot_variation`, `idle_shot_variation`, `delay_variation`) | Real effect, **fitted** size | Sized to match recordings. |

**dyno-sim:**
- **Petrol:** a Wiebe burn with literature parameters (burn duration against rpm, load and
  residuals), or a turbulent flame model that is validated rather than multiplied.
- **Diesel:** a heat release from an injection-rate profile (injector size, nozzle holes, rail
  or pump pressure) with ignition delay.
- **Calibration:** check against fuel consumption (BSFC around 200-250 g/kWh) instead of
  fitting torque. If torque then lands near the published figure, the models are right;
  `max_burning_efficiency`-style knobs shouldn't exist.

### Forced induction

This is where real specs can't be entered today. Everything below is **effective**.

| Part | What it is | What it isn't |
|---|---|---|
| Centrifugal compressor | A made-up Moore-Greitzer characteristic built from impeller tip radius, choke flow at max speed and one efficiency. Pressure scales with tip speed squared; the surge line sits at 30% of choke; duct inertia gives real surge and flutter dynamics. | No compressor map, no inducer or exducer and no trim. Efficiency is constant across the map. |
| Turbine | Torque = mass flow x radius x (a x spouting velocity - b x blade speed), peaking at a blade-speed ratio of about 0.7. | Its size is an orifice flow constant (`turbine_flow: k_carb(n)`), not A/R or a map. No pulse or twin-scroll behaviour, no exhaust energy in the manifold pulses. |
| Wastegate, blow-off valve | A pressure-set valve with a 40 kPa opening range. | No actuator spring or duty cycle. |
| Sequential twins | Exhaust valve pre-opening and a charge check valve. | Simplified control logic. |
| Series turbos (C15) | Compressors in series via an interstage volume, with the second stage's map scaled by inlet density. | **Turbines are in parallel** on the collector (the exhaust side is wrong), so the pair needs about one turbine's total area. |
| Roots blower | Displacement x speed x inlet density, less leakage, at an adiabatic efficiency. Displacement is real. | Leakage area is a guess. No bypass valve (Detroit 92TA). |

Every turbo in these files was sized by sweeping until boost and torque looked right. Rules
that came out of it, as starting points only:
- A four-stroke diesel's turbine runs `k_carb(170-260)` for 6-15 L.
- A **two-stroke's** turbine needs about **2x** that per litre (`~50` per litre), because the
  scavenging air goes straight through.
- A series pair (with parallel turbines) needs about one turbine's total area.

**dyno-sim**, in order of preference, mixable per part:
1. **Map-based:** compressor and turbine maps (digitized: pressure ratio against corrected
   flow, efficiency islands, speed lines). Many are published (Garrett, BorgWarner, Holset,
   IHI, MHI).
2. **Generated from geometry:** inducer and exducer diameters, trim, blade count, A/R, and
   turbine wheel diameter. Build the map from Euler work and slip, with an efficiency island
   placed by specific speed; set the turbine's flow capacity from A/R and the wheel. Enough for
   "an STI's VF-series turbo" when only the part number is known.
3. **Behaviour-defined:** target boost, spool speed and choke flow, solved backwards into
   (2)'s geometry. For "a turbo that behaves like X" when nothing is published.
4. **Plumbing:** turbines in series on the exhaust side, pulse-energy turbine inlet
   (twin-scroll and divided manifolds matter for the sound too), and a wastegate with spring
   preload and an actuator area.

### Two-strokes

All **effective**, and far from EngMod2T:

| Part | What it is | What it should be |
|---|---|---|
| Ports | A duration (total degrees, centred on BDC), an opening that grows in a straight line with piston travel below the port top, and one flow constant. | Port width, height, count, corner radii and angles, giving geometric area against crank angle and therefore blowdown time-area and transfer time-area. |
| Scavenging | One parameter: the gas leaving carries fresh share f^(1+q). | A real scavenging model: Benson-Brandham two-zone, or Sher's S-curve fitted per cylinder type (loop, cross, uniflow). |
| Crankcase | One lumped volume pumped by the piston. | Fine as 0-D. The transfer ducts should be 1-D. |
| Reed valve | A first-order lag on the pressure across it. | Petal stiffness, mass and stop plate: a spring-mass reed. |
| Disc valve | Open between two angles with fixed 15 deg ramps. | Real: area from disc cut-out geometry. |
| Power valve | A blend between two exhaust port timings by rpm. | Port height against blade position (KIPS, YPVS and AEC all differ). |
| Expansion chamber | Per-cell area from header, diffuser, belly, baffle and stinger, which is good, but see the pipe-length problem above. | The same, with wall heat loss, plus multi-stage cones. |
| Ring-ding | A misfire threshold on residual gas, tuned by ear. | Falls out of a real scavenging model plus a flammability limit. |

The Detroit two-strokes (uniflow, blower, head exhaust valves) use the same port model with
the crankcase off.

### Crank and mechanics

| Part | Status | Notes |
|---|---|---|
| Crank, rods, pistons | Physical | Upstream's rigid-body constraint solver (Gauss-Seidel). |
| Bank geometry | Physical | A cylinder reaches TDC at its pin angle **minus** its bank angle. Upstream's V examples don't compensate (four-strokes hide the error); the Detroit generator and the BeamNG generator do. |

**dyno-sim:** treat the crank as one rotating shaft with slider-crank kinematics per cylinder
(no constraint solver), plus torsional flex only if wanted for sound.

### Sound

Almost entirely **not acoustics**:

| Stage | What it is |
|---|---|
| Source | Pressure at each cylinder's exhaust pipe end, or the lumped primary. Not radiated sound. |
| Mix | `hf_gain` x d(pressure)/dt + (1 - `hf_gain`) x pressure x (`noise` x low-passed noise + 1 - `noise`). At upstream's `hf_gain` 0.01 the derivative is about 10x everything else, and `noise` 1.0 removes the plain pressure entirely: why every upstream engine is bright with no low end. The diesels here moved to 0.002 / 0.3. |
| Path | `primary_length` and exhaust `length` used as delays; per-cylinder `sound_attenuation`. |
| Muffler and stacks | Convolution with recorded impulse responses. `straight_stack_0` is upstream's `minimal_muffling_03` low-passed at 2.2 kHz; `diesel_muffler_0` is low-passed at 450 Hz. |
| Level | A loudness leveler (release time is a setting), so absolute loudness isn't physical. |
| Sound hacks | `manifold_relative_sound` (pressure minus the smoothed collector; it has no effect with pipes on, so it's dead in the Detroit files); `drone_filter` and `knock_resonance` (7.3 only); `combustion_noise` (a block channel with fixed resonances); the Jake brake's 100 Hz low-pass. |
| Turbo and blower sounds | Synthesized voices: whine at a blade-pass order (11-17x shaft speed), narrow-band air, hiss proportional to flow, surge puffs, wastegate and blow-off hiss, gear whine. Levels and tilt set by ear. |
| Missing | Radiation from the open pipe end, muffler and resonator acoustics, intake noise, mechanical noise (valvetrain, gears, injectors, piston slap), distance and ground reflection. |

**dyno-sim:**
- **Radiation:** radiate from the tailpipe's open end (a monopole from the time derivative of
  its mass flow).
- **Mufflers:** make them 1-D or 2-port acoustic elements, so a published muffler or a
  straight pipe is geometry, not a recorded response.
- **Other sources:** add the intake mouth and mechanical sources as separate, explicitly
  modelled sources.
- **Leveler:** keep one for the game, but outside the model.

## Calibration ledger: the engines this project added

What each was tuned with to land on its published figures, and what it still misses. Unless
noted, fuel and `max_burning_efficiency` were fitted to torque and turbo sizes to boost.

| Engine | Fitted to get there | Still off / notes |
|---|---|---|
| Mazda 13B-REW, 20B-REW (sequential) | Burn efficiency 0.67-0.70 (a large fudge), turbine sizes, sequential changeover | Burn efficiency that low means the model's breathing or burn is off for rotaries |
| Mazda 13B, bridgeport, 20B, 26B | Burn efficiency 0.93-0.97, port timing and sound by ear | |
| Toyota 2JZ-GTE | Flame gain 3, burn efficiency 0.88, turbine `k_carb(150)` | Lumped |
| Ford Voodoo | Flame gain 3.5, FMEP | **~448 Nm vs 582 real**: a breathing limit |
| Ford Predator (TVS) | Flame gain 3.5; TVS displacement real (2.65 L), ratio 1.8 | |
| Kawasaki Ninja H2 | Flame gain 6, supercharger ratio real (9.18), compressor guessed | **~116 Nm vs 142 real** |
| Subaru EJ257 STI | Flame gain 4, burn efficiency 0.97, turbine `k_carb(240)` | Compressor and turbine are not VF-series specs |
| Cummins 6BT | Fuel 66 mg, `manifold_relative_sound` 1.0 | Lumped |
| Detroit Series 60 | Fuel 250 mg, rated fuel 0.69, `manifold_relative_sound` 0.5 | Lumped; the head table is reused by the NTC-444, C15 and C16 |
| Ford 7.3 Power Stroke | Fuel 59 mg, `drone_filter` 1.2, `knock_resonance` 140, `combustion_noise` | Sound parked; the most sound-hacked file |
| Cummins NTC-444, CAT C15, CAT C16 | Fuel 190-300 mg, rated fuel 0.82-0.93, turbines `k_carb(260)`, `(220)`, `(130+90)`; S60 head x1.9 | NTC boost **~+220 kPa vs ~+170 real** |
| KDX220, Banshee, H2 750, RG500 | Flame gain 10, dilution 0.3, misfire residual 0.65, expansion chambers **1.3-1.6x long**, port timings, small idle air | Idle **~1850-2000 rpm** (high) |
| Detroit 6V/8V/12V/16V-71/-92 (generated by `tools/detroit_gen.py`) | Fuel per cylinder, turbines ~50/L, exhaust valve 105 deg at 0.050 in (chosen partly to match a recording), Roots leakage guessed | TA airbox boost **below** Detroit's spec (~+105-150 vs ~+170 kPa); blower bypass missing |

Diesel files also carry `fast_flame(gain: 1.5)`; it's unused (diesels don't use the flame
model).

## Inputs that mattered (sensitivity seen here)

The values people rarely know, and how much they moved things:
- **Pipe lengths and diameters:** decisive on two-strokes (the powerband) and audible on
  everything with pipes on.
- **Intake runner length:** with pipes on, a 2 in to 10 in airbox runner **halved** the
  Detroit's 400-800 Hz rasp (scavenging waves). With lumped intakes it does almost nothing.
- **Cylinder-to-cylinder differences** (runner lengths, injector bias): they fill the spectrum
  between harmonics. Too much and an engine sounds rough and smeared; matched cylinders give
  a clean bark.
- **Charge and interstage volumes:** spool feel and boost response.
- **Plenum volume:** little effect, but only because the lumped model has no waves to tune.
- **Turbine size:** dominant for torque, boost, back pressure and the blower's load (a
  two-stroke turbo sized like a four-stroke's cost the crank about 750 Nm).

## Reference data worth keeping

- **Head flow tables:** the GM LS (`atg-video-2/07_gm_ls.mr`), the most-scaled table; the
  Series 60 diesel table (`detroit/series_60.mr`).
- **Cam profiles:** upstream's real lobe profiles (e.g. Comp Cams Magnum lobes in the upstream
  files).
- **Recordings used for sound matching:**
  - Detroit Series 60 with 8 in straight pipes: `~/Kenworth 8 inch straight pipes 60 series detroit [HWCsIJ8V7NE].mp3`
  - 6V-92 straight-piped GMC Brigadier: `~/GMC Brigadier with straight piped Detroit at it again! [GK3p7Iwekjw].mp3` (idle about 24.5-26.5 s, load about 29.8-30.6 s; the mic cuts below ~45 Hz)
  - BeamNG's sample-based D-series diesel: `~/beamng.wav`
- **Analysis approach that worked:**
  - Order spectra: harmonics as multiples of the firing frequency.
  - Band shares.
  - The ratio of energy on the harmonics to energy between them (tonal against rough), per band.
  - Spectrograms against a reference.

  The scripts were throwaway, in /tmp; rebuild them in dyno-sim's tooling.

## Performance

- **Single core:** the whole simulation runs on one thread; audio is the only other thread.
- **Cost:** on the Detroit 16V-92TA, the 1-D pipes took about 40% of the time and the
  crankshaft's constraint solver about 25%. It ran at 0.7x real time.
  - A longer intake runner (bigger cells, fewer pipe steps) and 8 kHz brought it to about 1.2x.
  - The 12Vs run about 1.4x; six-cylinders 2-3x.
- **Pipe time step:** each pipe's step is set by its smallest cell and the hot gas behind an
  open valve. Short runners with a minimum of 4 cells are the worst case.
- **For dyno-sim:**
  - Cylinders and their pipes are independent between junctions, so they parallelize across
    cores.
  - A shaft model instead of a constraint solver frees about a quarter of the time.
  - A minimum cell size instead of a minimum cell count; local time stepping per pipe.
