# Drone Simulation and Control in MATLAB

A real-time quadrotor simulator: Newton–Euler rigid-body dynamics, a
cascaded position/attitude controller, 3D visualisation, and manual
flight from a keyboard or a joystick. Runs one drone with live telemetry,
or a nine-drone swarm holding a consensus formation.

The dynamic model implements:

> Teppo Luukkonen, *Modelling and control of quadcopter*,
> Aalto University, Espoo, 2011.

---

## Quick start

```matlab
cd Drone_Project

RunSingleDrone     % one drone with live telemetry
RunSwarm           % nine-drone formation
RunInputTest       % check your joystick/keyboard axis mapping
RunTests           % 55 checks on the model, mixer, integrator, controller
```

`cd` is not strictly required — each entry point puts `src/` on the path
itself, resolved relative to its own location.

### Flying

Press **M** to switch between AUTO and MANUAL at any time. In the swarm
simulation you fly the whole formation and the drones keep their shape
around you.

| Key | Action |
|---|---|
| `↑` / `↓` | pitch forward / backward |
| `←` / `→` | roll left / right |
| `W` / `S` | climb / descend |
| `A` / `D` | yaw left / right |
| `M` | toggle MANUAL / AUTO |
| `R` | reset |
| `Q` or `Esc` | quit |

A joystick is used automatically when one is present, otherwise the
keyboard. Manual flight is **angle mode**, like a real flight controller:
the sticks command a tilt *angle* and a climb *rate*, and the inner loop
keeps stabilising the airframe. Release everything and the drone holds
attitude and altitude.

### Options

```matlab
RunSingleDrone('mode', 'manual', 'duration', 60)
RunSingleDrone('input', 'keyboard')      % or 'joystick', 'auto'
RunSingleDrone('theme', 'light')         % default is dark
RunSingleDrone('layout', 'plus')         % rotor geometry, or 'x'
RunSwarm('numDrones', 5)

% Batch: no figure, as fast as the machine allows (~45x real time)
r = RunSingleDrone('headless', true, 'realTime', false);
```

`RunSingleDrone` returns a flight log with fields `t`, `x`, `e`, `omega`
and `mode`, sampled at 100 Hz.

---

## Structure

```
Drone_Project/
├── RunSingleDrone.m            one drone, manual + automatic flight
├── RunSwarm.m                  nine-drone consensus formation
├── RunInputTest.m              live joystick/keyboard axis diagnostic
├── RunTests.m                  all 55 verification checks
├── src/
│   ├── sim_QuadrotorPlant.m    rigid-body dynamics, mixer, actuators
│   ├── sim_CascadeController.m position -> attitude -> torque loops
│   ├── sim_PilotInput.m        reads the keyboard or joystick
│   ├── sim_Window.m            the figure: 3D view + telemetry plots
│   ├── sim_DroneGraphic.m      one drone's 3D mesh (shared STL cache)
│   ├── sim_FixedStepClock.m    real-time pacing and loop scheduling
│   ├── sim_FormationPlanner.m  consensus formation + collision avoidance
│   ├── sim_droneParams.m       physical constants, each tagged with source
│   ├── sim_config.m            every loop rate, stated once
│   ├── sim_rk4Step.m           one Runge-Kutta step
│   ├── sim_wrapAngle.m         toolbox-free wrapToPi
│   └── sim_saturate.m          clamp to an interval
└── media/                      STL models
```

### Naming

**1. The prefix says whether you type it.**

| Prefix | Meaning |
|---|---|
| `Run…` | an entry point — something you run |
| `sim_…` | internal to the simulator — you never type it |

The root holds exactly the four things you can run. Everything else is
one folder down.

**2. The character after `sim_` says what kind of thing it is.**

| Form | Kind | Examples |
|---|---|---|
| `sim_PascalCase` | a class | `sim_QuadrotorPlant`, `sim_FixedStepClock` |
| `sim_camelCase` | a function | `sim_rk4Step`, `sim_wrapAngle` |

So `sim_FormationPlanner` is a class and `sim_config` is a function, and
you can tell without opening either. Names state the **role** rather than
the domain — everything here is about drones, so "drone" in a name would
carry no information. A file's name matches its own first documentation
line; all 16 do.

---

## The model

State vector, 12 elements:

```
 1: 3   x, y, z            inertial position       [m]
 4: 6   xdot, ydot, zdot   inertial velocity       [m/s]
 7: 9   psi, theta, phi    yaw, pitch, roll        [rad]
10:12   p, q, r            body angular rates      [rad/s]
```

Note that the angles run yaw-pitch-roll while the rates run
roll-pitch-yaw, so `x(7)` and `x(12)` are the same axis, and `x(9)` and
`x(10)` are the same axis.

`sim_QuadrotorPlant` implements, with the paper's equation numbers:

| Equation | What it covers |
|---|---|
| Eq. (21) | translational dynamics with aerodynamic drag `-(1/m)·A·ξ̇` |
| Eq. (11) | body angular accelerations, including the rotor gyroscopic term |
| Eq. (4)  | Euler-rate kinematics `η̇ = W_η⁻¹ν`, guarded against gimbal lock |
| Eq. (7), (8) | thrust and torques from rotor speeds |

Integration is RK4 at 500 Hz, in `sim_rk4Step`.

### Parameters

Every constant comes from Table 1 of the paper:

| Symbol | Value | Unit | | Symbol | Value | Unit |
|---|---|---|---|---|---|---|
| `g`  | 9.81 | m/s² | | `Ix` | 4.856·10⁻³ | kg m² |
| `m`  | 0.468 | kg | | `Iy` | 4.856·10⁻³ | kg m² |
| `l`  | 0.225 | m | | `Iz` | 8.801·10⁻³ | kg m² |
| `k`  | 2.980·10⁻⁶ | — | | `Ax` | 0.25 | kg/s |
| `b`  | 1.140·10⁻⁷ | — | | `Ay` | 0.25 | kg/s |
| `Ir` | 3.357·10⁻⁵ | kg m² | | `Az` | 0.25 | kg/s |

Values *not* in the paper are tagged `[ENG]` in `sim_droneParams.m` with
their justification inline: the rotor speed ceiling (950 rad/s, giving a
thrust-to-weight ratio of 2.34), an idle floor at 15 % of hover, and a
30 ms first-order motor lag. Set `motorTau = 0` for the paper's ideal
actuator.

The hover speed is **derived**, not hard-coded: `wHover = sqrt(m·g/4k)`
= 620.6108 rad/s, so it cannot fall out of step if a mass or lift
constant is edited.

### Rotor geometry and mixing

The allocation matrix is *built* from the rotor positions and spin
directions rather than written out by hand:

```
[T; τφ; τθ; τψ] = F · [ω₁²; ω₂²; ω₃²; ω₄²]
```

The gyroscopic term uses the same spin vector, `ωΓ = Σ sᵢ·ωᵢ`, so the
yaw torque signs and the gyroscopic coupling cannot drift out of
agreement. Two layouts are available, both exact:

- `droneParams('x')` — rotors on the 45° diagonals, matching the STL
  airframe, each at distance `l` from the centre of mass so the per-axis
  moment arm is `l/√2`. **Default.**
- `droneParams('plus')` — rotors on the body axes, reproducing Eq. (8)
  of the paper term for term.

The mixer desaturates by priority — **thrust, then roll/pitch, then
yaw** — so an infeasible command loses heading authority before it loses
the ability to stay upright, and the commanded tilt direction survives
even when its magnitude cannot. This airframe makes only about
0.086 N·m of yaw torque at hover thrust, so yaw saturates routinely.

### Simplifications

Worth knowing when interpreting results. From the paper:

- Thrust is independent of angle of attack and airspeed; blade flapping
  and airflow disruption are excluded.
- Drag is linear in velocity. `Ax, Ay, Az` were *chosen* so the
  quadcopter slows and stops when levelled, not measured.
- The inertia matrix is diagonal and constant.

Added here: an inelastic ground plane at `z = 0`, and rotor speed limits.
Not modelled: wind, sensor noise, or a state estimator — the controller
has perfect state feedback.

---

## Control

Three loops, each clearly slower than the one inside it:

| Loop | Rate | Bandwidth |
|---|---|---|
| Position → tilt reference | 100 Hz | ωₙ = 1.5 rad/s |
| Attitude + altitude → torques | **500 Hz** | ωₙ = 12 rad/s (5 for yaw) |
| Mixer → rotor speeds | 500 Hz | — |

The control law is Eq. (23) of the paper, including its inertia
normalisation:

```
τφ = (Kd·(φ̇d − φ̇) + Kp·(φd − φ)) · Ixx
T  = (g + Kd·(żd − ż) + Kp·(zd − z)) · m / (cos φ · cos θ)
```

That normalisation is what makes the gains readable: for a second-order
loop `Kp = ωₙ²` and `Kd = 2ζωₙ` directly. `sim_CascadeController` stores
them as `(ωₙ, ζ)` pairs and converts, so the separation between loops is
visible by inspection. Small integral terms with clamped anti-windup sit
on altitude and horizontal position; set `kiAlt` and `kiPos` to 0 for the
paper's pure-PD behaviour.

The derivative term acts on Euler rates (`W_η⁻¹ν`), not raw body rates —
those coincide only near hover.

### The simulation loop

`sim_FixedStepClock` is a fixed-timestep accumulator. The wall clock
decides how many physics steps are owed, each step is exactly `dt`, and
every subsystem has its own divider:

| Subsystem | Rate | Divider |
|---|---|---|
| Plant (RK4) | 500 Hz | — |
| **Attitude + altitude + mixer** | **500 Hz** | **1 — every physics step** |
| Position loop | 100 Hz | 5 |
| Input poll | 50 Hz | 10 |
| Logging | 100 Hz | 5 |
| 3D render | 50 Hz | 10 |
| Telemetry plots | 10 Hz | 50 |
| Heads-up text | 5 Hz | 100 |

The inner control loop runs **in lockstep with the plant** — one control
update per integration step, not a zero-order hold wrapped around a
faster plant. Rendering is decoupled from control, so a slow frame costs
frames rather than control accuracy. Simulated time advances in exact
multiples of `dt`; lag is measured and reported rather than hidden, and
each run prints its real-time factor on exit:

```
Simulated 30.00 s in 30.02 s of wall clock (1.00x real time, 15000 steps, 0 resyncs).
```

Every rate lives in `sim_config.m`.

---

## Formation control

`sim_FormationPlanner` runs a weighted consensus over each drone's
visible neighbours:

```
xᵢ_des = Σⱼ cᵢⱼ · (xⱼ + sᵢ − sⱼ),    Σⱼ cᵢⱼ = 1
```

where `sᵢ` is drone `i`'s offset from the formation centroid. The
formation is an exact fixed point of this law: at `xⱼ = x_c + sⱼ` every
term equals `x_c + sᵢ`. Weights are normalised over exactly the set that
is summed — self plus visible neighbours — with extra confidence on the
leader when it is in range. A drone that loses contact steers to its
assigned slot around the leader's goal rather than freezing.

Collision avoidance adds a repulsive offset that ramps linearly to zero
at a 1.2 m influence radius, applied to velocity-extrapolated positions
so it reacts to closing speed rather than only to current separation.

In manual mode the pilot moves the formation centroid and the swarm
keeps its shape around it. The heads-up text reports RMS distance to the
commanded slots and the closest pair in the swarm.

---

## Tests

```matlab
RunTests
```

55 checks in four suites, no figures and no toolboxes, in a few seconds:

- **modelChecks** — Table 1 constants, hover equilibrium, the drag
  `1/m`, thrust direction against `R·e_z` at arbitrary attitude, the
  Euler-rate transform, both gyroscopic terms, centripetal terms, free
  fall, and the gimbal-lock guard.
- **mixerChecks** — Eq. (8) term by term for the plus layout, rotor
  distance from the centre of mass, spin balance, round-trip exactness,
  desaturation preserving roll/pitch direction, yaw yielding first,
  envelope clamping, thrust-to-weight plausibility.
- **integratorChecks** — measured convergence order (4.00 for RK4,
  1.00 for Euler) and hover holding over 2 s of integration.
- **controlChecks** — bandwidth separation, control-rate adequacy, hover
  trim, and a full headless closed-loop run asserting convergence,
  envelope compliance, tilt limits, overshoot, and yaw taking the short
  way round.

The suite runs the real loop at the shipped rates, so it exercises the
code that actually flies rather than a simplified stand-in.

---

## Customisation

- **Controller gains** — `src/sim_CascadeController.m`, as `(ωₙ, ζ)`
  pairs.
- **Physical parameters** — `src/sim_droneParams.m`. Paper values are
  tagged with their table or equation; engineering additions are tagged
  `[ENG]`.
- **Rotor geometry** — `droneParams('x')` or `droneParams('plus')`.
- **Loop rates** — `src/sim_config.m`.
- **Joystick axis mapping** — `AXIS_MAP` / `AXIS_SIGN` in
  `src/sim_PilotInput.m`; run `RunInputTest` to see live values.

## Possible extensions

- Sensor models and a state estimator, so the controller stops getting
  perfect feedback.
- Wind and gust disturbances.
- Trajectory tracking using the exact tilt inversion of Eq. (26) rather
  than the small-angle map.
- Quadratic drag and thrust dependence on airspeed.

---

## Requirements

- MATLAB R2019b or later (`arguments` blocks). Developed on R2025b.
- No toolboxes required.
- Optional: a joystick via `sim3d.io.Joystick` (Simulink 3D Animation /
  UAV Toolbox).
- The dark theme needs R2025a or later; older releases fall back
  silently.

## License

MIT.

## Acknowledgments

- Dynamic model and control structure from *Modelling and control of
  quadcopter* by Teppo Luukkonen.
- STL models provided for educational purposes.
