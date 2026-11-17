# AGENTS.md: DroneSim_Simple (quad-X drone, built one stage at a time, no motor drive)

You are Codex running **online (cloud sandbox) on this git repository**. This repo is the whole project.
There is no MATLAB, no Simulink and no access to my computer in your sandbox. So the work is split:

- **You (cloud):** write the MATLAB build and test scripts, AND a separate **Python reference model** that
  you can actually run. The Python model proves every number and design choice before I touch MATLAB.
- **Me (human):** I pull your commit, run one MATLAB command on my machine (R2023a, Simulink), and paste
  the console output back to you in the next task. You fix whatever failed.

Use only paths relative to the repo root. Never refer to `F:\` or any machine-specific path. Do not touch
anything outside this repo.

## Honesty rule (most important)

You cannot run MATLAB or Simulink. So:
- A number is **"verified here"** only if a Python test you ran in this sandbox produced it. Quote the pytest output.
- A MATLAB script is **"written, not run"** until I paste back a passing result. Never claim a Simulink model
  works, compiles, or passes a test. Never present an unverified number as a result.
- If a test fails, find the cause. Never loosen a pass condition to get a pass. If my hand calculation is
  wrong, show the corrected one.

## Repo layout (create in S0)

```
AGENTS.md            this file
STATUS.md            hand-off file: stage table, what I must run next, results I pasted back, open issues
requirements.txt     numpy, scipy, pytest, matplotlib
python/dronesim/     Python reference model: params.py, body.py, mixer.py, motors.py, control.py, mission.py, sim.py
tests/py/            pytest tests, one file per stage (test_stage1.py ...)
expected/            stageN.json: numbers the Python model produced, which the MATLAB tests compare against
matlab/              drone_parameters.m, build_stageN.m, test_stageN.m, run_stage.m, check_layout.m, export_diagrams.m
results/diagrams/    PNGs I export locally (you do not create these)
DroneSim.slx         created by matlab/build_stage0.m on my machine (not committed by you)
```

## Every task (one stage per task)

1. Read `AGENTS.md`, `STATUS.md`, `matlab/drone_parameters.m`, and any output I pasted in the task text.
2. If I pasted a failure from the previous stage, **fix that first**, then continue.
3. Do only the stage I name. Do not start the next stage.
4. Deliverables for stage N:
   - Python reference for that stage in `python/dronesim/`, plus `tests/py/test_stageN.py`. Run `pytest -q`
     and make everything pass. Print `expected | got | PASS/FAIL` lines in the test output.
   - `expected/stageN.json` written by the Python tests (keys, values, tolerances).
   - `matlab/build_stageN.m`: adds this stage to `DroneSim.slx`; re-runnable (delete and recreate its own
     subsystem first); wraps each block in try/catch and prints which block failed.
   - `matlab/test_stageN.m`: runs the Simulink model, loads `expected/stageN.json` with `jsondecode`, prints
     `expected | got | PASS/FAIL` for every line.
   - Update `matlab/drone_parameters.m` (flat names, rules below) and `expected/params.json` (every parameter
     the Python model uses, so `matlab/test_params.m` can check the two files agree).
   - Update `STATUS.md`.
5. Final message (short): what was built block by block in 3 to 5 plain-English lines (I am learning this),
   the pytest output (verified here), the exact MATLAB command I should run (`run_stage(N)`), deviations and
   why, questions. Ask a question only if you are blocked; otherwise make a judgment call and state it.

`matlab/run_stage.m` (written in S0): `run_stage(N)` runs `build_stageN`, `test_params`, `test_stageN`,
`check_layout`, `export_diagrams`, then prints one summary table. I paste that output back.

## MATLAB scripting rules (you write these blind, so be careful)

- Idempotent: each build script deletes and recreates only what it owns.
- Place every block with an explicit `Position` from a layout helper (grid in `matlab/`): one row per idea, left
  to right, fixed column x-positions. Use `add_line(sys,'A/1','B/1')` with straight or one-bend routes. Do not
  rely on autorouting.
- Block paths valid in R2023a, e.g. `simulink/Math Operations/Sum`, `simulink/Math Operations/Gain`,
  `simulink/Continuous/Integrator`, `simulink/Discontinuities/Saturation`,
  `simulink/Signal Routing/Bus Creator`, `simulink/Signal Routing/Bus Selector`,
  `simulink/User-Defined Functions/MATLAB Function`, `simulink/Discontinuities/Rate Limiter`.
- A MATLAB Function block's code is set with `Stateflow.EMChart`
  (`rt = sfroot; ch = rt.find('-isa','Stateflow.EMChart','Path',blk); ch.Script = code;`). Parameters it uses are
  added as `Stateflow.Data` with `Scope = 'Parameter'`.
- Do not require any toolbox beyond Simulink. No `linearize`, no Control System Toolbox: loop margins are
  computed in Python from the plant equations; MATLAB tests check step responses against the Python result.
- Solver: fixed-step `ode4`, step 1.25 ms. Stop time is typed in the Simulink toolbar (no `SIM` variables).
- The model's InitFcn runs `drone_parameters`; PreLoadFcn adds `matlab/` to the path.

## Parameters: one file, `matlab/drone_parameters.m`

- Flat names, no structs (no `VEH.mass`). Examples: `drone_mass`, `kT_prop`, `Kp_velocity`.
- Every line: `name = value;  % unit, meaning, source or PLACEHOLDER`.
- Sections in this order, each with a one-line header comment:
  1 Mission table, 2 Drone, 3 Propeller, 4 Motor (ideal), 5 Flight controller, 6 Derived values.
- Derived values: one formula per line with a comment saying why. No anonymous functions. A gain chosen by
  crossover is shown as named steps: crossover frequency -> plant gain at crossover -> Kp.
- `tilt_compensation = 1;  % 1 on, 0 off`. The drone starts airborne in hover at 5 m (say so in the file).
- Under about 150 lines. Never hard-code a number inside a block or in a test; use the parameter name.
- Python keeps the same names in `params.py`.

## Layout rules (every Simulink level)

1. One idea per row, left to right: `reference -> Sum -> PI -> gain -> limit -> output`. Roll, pitch, yaw (or
   x, y, z) are stacked rows in the same layout.
2. No Demux of the state vector. Each row has its own Bus Selector directly below its Sum, selecting only what
   that row needs; feedback enters the Sum from below.
3. No crossing lines, no overlapping blocks, at most about 20 blocks per level (split into named subsystems).
4. Every line named with units; plain-English block names; placeholder blocks coloured light orange.
5. `matlab/check_layout.m` counts line crossings (from each line's `Points`), overlapping blocks and blocks per
   level for every level. Target: 0 crossings, 0 overlaps. You write it in S0; I run it.

## Stage list

| Stage | Build |
|---|---|
| S0 | repo skeleton, run_stage / check_layout / export_diagrams, empty model with solver settings |
| S1 | drone body (6-DOF) and the `states` bus |
| S2 | thrust to speed, ideal motors, mixer |
| S3 | rate control |
| S4 | attitude control |
| S5 | velocity and altitude control |
| S6 | mission command |
| S7 | top-level assembly, scopes, flight path |
| S8 | full-system tests and independent recheck |

I name the stage in each task. Never do more than one.

## S0. Skeleton

Create the repo layout, `requirements.txt`, `STATUS.md`, the MATLAB helper scripts, `build_stage0.m` (empty
`DroneSim.slx` with InitFcn / PreLoadFcn / solver), `matlab/drone_parameters.m` containing only the 6 section
headers and `tilt_compensation = 1;`, and a pytest smoke test. Check in this sandbox: `pytest -q` passes and
`python -c "import dronesim"` works.

## S1. Drone body (6-DOF quad X)

Frames: NED (z down), body x forward, y right, z down. Subsystem `Drone body`: Inports `motor_speed` (4x1,
rad/s) and `motor_torque` (4x1, N m); MATLAB Function `Dynamics`; Integrator with initial condition
`initial_state`; Demux (12) -> Bus Creator (12) -> Outport `states` with elements
`north, east, down, v_north, v_east, v_down, roll, pitch, yaw, p, q, r`.

Add to the parameter file:

```matlab
%% 2. Drone
gravity          = 9.81;          % m/s^2, gravitational acceleration
drone_mass       = 300;           % kg, total mass, PLACEHOLDER
inertia_xx       = 100;           % kg m^2, roll inertia, PLACEHOLDER
inertia_yy       = 100;           % kg m^2, pitch inertia, PLACEHOLDER
inertia_zz       = 185;           % kg m^2, yaw inertia, PLACEHOLDER
arm_length       = 1.1*sqrt(2);   % m, centre to each motor, PLACEHOLDER
motor_x          = arm_length/sqrt(2)*[ 1 -1  1 -1];  % m, forward position of motors 1..4 (FR, BL, FL, BR)
motor_y          = arm_length/sqrt(2)*[ 1 -1 -1  1];  % m, right position of motors 1..4
motor_spin_dir   = [1 1 -1 -1];   % +1: reaction torque turns the nose right
drag_linear      = 20;            % N/(m/s), body drag, PLACEHOLDER
drag_rotational  = 50;            % N m/(rad/s), body rotational drag, PLACEHOLDER
ground_stiffness       = 3e5;     % N/m, ground spring, PLACEHOLDER
ground_damping         = 2*sqrt(ground_stiffness*drone_mass);  % N s/m, critically damped
ground_side_friction   = 5e3;     % N s/m, sideways friction on the ground
ground_level_stiffness = 2e4;     % N m/rad, ground levels the body
ground_level_damping   = 2e3;     % N m s/rad
initial_state    = [0;0;-5; zeros(9,1)];  % drone starts airborne in hover at 5 m (NED, z down)

%% 3. Propeller
kT_prop = 0.01293;  % N/(rad/s)^2, thrust = kT*w^2
kQ_prop = 9e-4;     % N m/(rad/s)^2, drag torque = kQ*w^2, PLACEHOLDER
J_prop  = 0.7;      % kg m^2, propeller inertia, PLACEHOLDER

%% 6. Derived values
hover_speed = sqrt(drone_mass*gravity/4/kT_prop);  % rad/s, 4 props lift the weight (238.5)
```

`Dynamics` (parameters used are Parameter data; sizes 4x1, 4x1, 12x1 -> 12x1). `python/dronesim/body.py`
implements the same equations line for line:

```matlab
function xdot = fcn(motor_speed, motor_torque, x)
% x = [north east down; v_north v_east v_down; roll pitch yaw; p q r]
v = x(4:6); phi = x(7); th = x(8); psi = x(9); w = x(10:12);
T = kT_prop * motor_speed(:).^2;
M = [-sum(motor_y(:).*T); sum(motor_x(:).*T); sum(motor_spin_dir(:).*motor_torque(:))];
cf = cos(phi); sf = sin(phi); ct = cos(th); st = sin(th); cp = cos(psi); sp = sin(psi);
R = [ct*cp, sf*st*cp - cf*sp, cf*st*cp + sf*sp;
     ct*sp, sf*st*sp + cf*cp, cf*st*sp - sf*cp;
     -st,   sf*ct,            cf*ct];
F = R*[0; 0; -sum(T)] + [0; 0; drone_mass*gravity] - drag_linear*v;
if x(3) > 0   % on or below the ground
    F(3)   = F(3) - max(0, ground_stiffness*x(3) + ground_damping*v(3));
    F(1:2) = F(1:2) - ground_side_friction*v(1:2);
    M = M - [ground_level_stiffness*phi + ground_level_damping*w(1);
             ground_level_stiffness*th  + ground_level_damping*w(2);
             ground_level_damping*w(3)];
end
I = diag([inertia_xx inertia_yy inertia_zz]);
wdot = I \ (M - cross(w, I*w) - drag_rotational*w);
eulDot = [w(1) + sf*st/ct*w(2) + cf*st/ct*w(3);
          cf*w(2) - sf*w(3);
          (sf*w(2) + cf*w(3))/ct];
xdot = [v; F/drone_mass; eulDot; wdot];
```

Tests (Python first, then the same in MATLAB; body alone, constant inputs):

| Test | Inputs | Expected |
|---|---|---|
| hover | `motor_speed = hover_speed*ones(4,1)`, `motor_torque = kQ_prop*hover_speed^2*ones(4,1)` | `down` stays -5.000 m, every other state 0 |
| motors off, drag off | both 0, `drag_linear = 0` for this test only, 1 s | falls 4.905 m |
| motors off, drag on | both 0, `drag_linear = 20`, 1 s | falls about 4.80 m; hand formula (g/k)(t-(1-e^(-kt))/k), k = drag_linear/drone_mass |
| pitch | motors 1 and 3 (front) thrust `drone_mass*gravity/4 - 10`, motors 2 and 4 (rear) `+10`, `motor_speed = sqrt(T/kT_prop)`, hover torques | moment 1.1 x (-40) = -44 N m, qdot = -0.44 rad/s^2 at t=0, height holds at first |
| yaw | hover speeds, `motor_torque = [5;5;0;0]` | rdot = 10/inertia_zz = 0.0541 rad/s^2, no roll or pitch |
| ground | motors off, start at `down = 0` with zero velocity | settles at `drone_mass*gravity/ground_stiffness` with no overshoot, within about 0.5 s |

## S2. Thrust to speed, ideal motors, mixer

The saved local `DroneSim.slx` is the source of truth for completed stages. S2 requires the existing
`Drone body` and preserves its blocks and wiring. The S2 runner tests S1 before S2.

Add to the parameter file:

```matlab
%% 3. Propeller (add)
max_speed        = 3000*pi/30;   % rad/s, 3000 rpm: keeps kQ*w^2 under the 96 N m continuous torque
%% 4. Motor (ideal speed lag, EMRAX 228 data for the limits)
pole_pairs       = 10;           % pole pairs
flux_linkage     = 0.0542;       % V s, magnet flux
J_rotor          = 0.0383;       % kg m^2, rotor inertia
I_max            = 340/1.5;      % A peak, current limit (226.7 A)
motor_time_constant = 0.0083;    % s, speed lag standing in for a motor drive, PLACEHOLDER
%% 5. Flight controller (add)
mixer_matrix     = [-1  1  1;    % columns: roll, pitch, yaw demand; rows: motors 1..4 (FR, BL, FL, BR)
                     1 -1  1;
                     1  1 -1;
                    -1 -1 -1];
%% 6. Derived values (add)
max_thrust       = kT_prop*max_speed^2;                 % N per motor at command 1 (1276)
hover_command    = drone_mass*gravity/(4*max_thrust);   % 0..1, command where 4 props lift the weight (0.5766)
torque_constant  = 1.5*pole_pairs*flux_linkage;         % N m/A, amplitude-invariant dq (0.813)
J_total          = J_rotor + J_prop;                    % kg m^2, rotor + prop (0.738)
max_motor_torque = torque_constant*I_max;               % N m at the current limit (184.3)
speed_slew_limit = 0.8*(max_motor_torque - kQ_prop*max_speed^2)/J_total;  % rad/s^2, 20 % margin (103)
```

- `Thrust to speed`: Inport `motor_cmd` (4x1, 0..1) -> Saturation 0..1 -> Gain `max_thrust/kT_prop` ->
  Math Function sqrt -> Rate Limiter (rising and falling `speed_slew_limit`, initial value `hover_speed`)
  -> Outport `motor_speed_ref`.
- `Motors (ideal, placeholder)`: Inport `motor_speed_ref`; Sum(ref - w) -> Gain `1/motor_time_constant` = `dw`
  -> Integrator (initial `hover_speed*ones(4,1)`) = `motor_speed`; MATLAB Function
  `motor_torque = kQ_prop*w.*abs(w) + J_total*dw`; Outports `motor_speed`, `motor_torque`.
- `Mixer`: Inports `collective` (1) and `axis_demand` (3x1: roll, pitch, yaw); Gain `mixer_matrix`
  (matrix multiply) plus Gain `ones(4,1)` on collective -> Sum -> Saturation 0..1 -> Outport `motor_cmd`.

Tests:

| Block | Input | Expected |
|---|---|---|
| thrust to speed | 0.5766 | 238.5 rad/s |
| thrust to speed | 0, 1, 1.2, -0.2 (at least 1 s each, slew limit) | 0, 314.2 (3000 rpm), 314.2, 0 |
| mixer | collective 0.5766, demand [0.1;0;0] | [0.4766 0.6766 0.6766 0.4766], sum unchanged |
| mixer | demand [0;0.1;0] | [0.6766 0.4766 0.6766 0.4766] |
| mixer | demand [0;0;0.1] | [0.6766 0.6766 0.4766 0.4766] |
| mixer | collective 0.95, demand [0.1;0;0] | motors 2 and 3 clip at 1 |
| ideal motor, slew limiter bypassed | +2 rad/s speed-reference step | reaches 63 % at 8.3 ms |
| ideal motor | reference = hover speed | holds 238.5 rad/s, `motor_torque` = 51.2 N m |
| mixer + thrust to speed + motors + body | positive roll, pitch, yaw demands | roll accel positive for positive roll demand (left motors 2, 3 up), pitch accel positive for positive pitch demand (front motors 1, 3 up), yaw accel positive for positive yaw demand |

## S3. Rate control (roll, pitch, yaw rate)

Subsystem `Rate control`, three stacked rows. Inputs: rate demands (3x1) and `states` (p, q, r via Bus
Selectors). Output: `axis_demand` (3x1, normalised, 1 = max thrust per motor). Each row:
`rate demand -> Sum -> PI with clamp / anti-windup -> output limit`.

Design method (named steps in the parameter file, computed and checked in Python first):
- plant gain from axis command to angular acceleration: roll and pitch `K = 4*a*max_thrust/inertia`
  (a = arm_length/sqrt(2)); lag `1/(motor_time_constant*s + 1)`; plant `K/s` times the lag
- crossover frequency chosen (start: roll and pitch rate 8 rad/s, PI zero at crossover/4, r = 0.25)
- `Kp = 1/(K*|G(j*wc)|*sqrt(1+r^2))`, `Ki = Kp*r*wc`, where `|G(j*wc)| = 1/(wc*sqrt(1+(motor_time_constant*wc)^2))`
- **yaw is special:** with the propeller inertia, the yaw plant is a gain (not an integrator) above about
  0.6 rad/s, so use an integral-dominant PI: `Ki = wc/|Gyaw(j*wc)|` with wc about 2 rad/s and a small Kp
  (start 0.3). Derive `Gyaw` yourself from `J_total`, `kQ_prop`, `hover_speed`, `inertia_zz` and show it.
- starting output limits (normalised): roll and pitch 0.15, yaw 0.08; integrator limits 0.10 and 0.08

Python tests, per axis: crossover and phase margin from the frequency response (numpy); require phase margin
at least 45 degrees and crossover within 20 % of design. Closed-loop simulation in the Python model: 0.05 rad/s
rate step (rise time, overshoot, zero steady error) and a step large enough to hit the output limit (no
windup, overshoot after recovery under 30 %). MATLAB test: the same steps on the Simulink model, compared with
the Python numbers (rise time within 15 %, overshoot within 5 points). State which numbers depend on placeholders.

## S4. Attitude control (roll and pitch angle)

Subsystem `Attitude control`: roll and pitch each `angle demand -> Sum (Bus Selector for roll or pitch below
it) -> P gain -> rate limit -> rate demand`. The mission gives a yaw RATE, so the yaw rate demand passes
straight to the rate loop (no yaw-angle loop). Report heading drift over a 3 s hover gap in the Python model;
if it exceeds 0.5 degrees, say so and propose a fix but do not add it without my go.

Design: attitude crossover about one quarter of the rate crossover (about 2 rad/s), `P = crossover`.
Add `max_tilt_deg = 20;  % deg, tilt limit` to the parameter file.

Tests: 5 degree roll and pitch steps: rise time vs 1/crossover, overshoot under 10 %, zero steady error, phase
margin (Python), rate loop stays inside its limits. MATLAB compares step responses with Python.

## S5. Velocity and altitude control

Three stacked rows (north, east, climb): `reference -> Sum (error) -> PI + clamp -> unit gain -> limit`. Each
row reads only its own measurement by a Bus Selector below its Sum (`v_north`, `v_east`, `-v_down`). The climb
row also has the altitude hold (outer P loop on `-down`).

- Horizontal rows: the PI gives an acceleration demand (m/s^2); the unit gain is `1/gravity`; the limit is
  `max_tilt_deg`. Rotate the NED acceleration demand into body x and y with the current yaw (one small clean
  block), then roll demand `+a_y/gravity`, pitch demand `-a_x/gravity`. Show your sign check in Python.
- Climb row: the PI gives a collective increment added to `hover_command` (hover feedforward). With
  `tilt_compensation = 1` divide the collective by `cos(roll)*cos(pitch)`; with 0, do not.
- Starting design (named steps): horizontal velocity crossover about 0.8 rad/s, PI zero at 0.1 of it; vertical
  velocity crossover about 2.5 rad/s with plant gain `4*max_thrust/drone_mass` and the motor lag; altitude
  P = vertical crossover / 4.
- Every PI has a clamp and anti-windup.

Tests: 1 m/s velocity step on each axis and a 1 m altitude step: closed-loop time constant vs design, overshoot
under 10 %, zero steady error; tilt demand stays inside `max_tilt_deg`; with tilt compensation OFF show the
altitude loss in a forward-velocity step and with it ON show it is smaller. Python first, MATLAB compares.

## S6. Mission command

Block `Mission command` reads the table in section 1 of the parameter file:
`% time[s]  forward[m/s]  right[m/s]  up[m/s]  yaw_rate[deg/s]` and outputs the references. Eight
manoeuvres, each on its own: forward, back, right, left, climb, descend, yaw right, yaw left. Opposite pairs
have the same size and duration; a hover gap of at least 3 s after each manoeuvre; total under about 60 s;
no Mux. **Propose the sizes in your task report and wait for my go before building the MATLAB part.**
Suggested: horizontal 2 m/s for 5 s, vertical 1 m/s for 3 s (climb before descend, never below 2 m), yaw
10 deg/s for 3 s. The table is in body forward/right; rotate to north/east with the current yaw and state this.

Tests: table sanity (pairs mirror, gaps at least 3 s, total time), block output at 5 sample times equals the
table, distance per manoeuvre = speed x duration.

## S7. Top level, scopes, flight path

One row, left to right: `Mission command -> Velocity and altitude control -> Attitude control -> Rate control
-> Mixer -> Thrust to speed -> Motors (ideal, placeholder) -> Drone body`. One feedback bus `states` runs along
the bottom. `Waveforms` and `Flight path` blocks at the bottom with 3 lines of notes.

Scopes (all with "Open at simulation start" OFF; one row per scope: `From -> unit Gain -> Scope`): flight
(velocity vs command, altitude, attitude vs demand, body rates vs yaw command), motors (mixer output, w and
w_ref in rpm, differential speed reference, Te, shaft power), command, ground track (XY graph, East
horizontal, North vertical), 3D path drawn when the run ends. Note at the bottom: "Set the stop time in the
toolbar, press Run, then double-click Waveforms."

Tests: `check_layout` on every level (0 crossings, 0 overlaps), model compiles, 5 s hover. These can only be
run by me, so write them clearly and wait for my pasted result.

## S8. Full-system tests and independent recheck

| Test | Expected |
|---|---|
| 5 s hover | holds 5 m, 2278 rpm, 51.2 N m per motor, shaft power 12.2 kW per motor (48.9 kW total) |
| each of the 8 manoeuvres alone | distance about speed x duration (state the lag allowance); settles in the hover gap |
| opposite pairs | mirror-image motor speed changes |
| full mission | path returns to the start within 0.5 m |
| mass +20 % (360 kg) | 2278 x sqrt(1.2) = 2495 rpm, 51.2 x 1.2 = 61.4 N m per motor |
| tilt compensation off | altitude loss in forward flight, reported |

Run the Python full-mission simulation here (verified here). Then write `matlab/test_stage8.m` to produce the
same numbers from the Simulink model, and a compare table. For the independent recheck, write
`tests/py/recheck_stage8.py` that recomputes every reported number from hand formulas only, sharing no code
with `python/dronesim/`; list any disagreement. The Simulink side stays "written, not run" until I paste results.

## Out of scope for now

Motor drive control, battery and DC bus, averaged and switching inverter models, and powergui. The
`Motors (ideal, placeholder)` block keeps the port names `motor_speed_ref`, `motor_speed`, `motor_torque` so a
drive can replace it later.