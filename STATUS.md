# DroneSim_Simple handoff

The build and reference scripts now cover S0–S8. The complete 52 s mission and independent S8 replay passed in Python. The user confirmed S0 and the original S1 in MATLAB R2023a. The revised ground case and S2–S8 are written, not run in MATLAB here.

## Run the final validation

Pull main. Set MATLAB's current folder to the repository root containing your saved DroneSim.slx and run

```matlab
addpath('matlab'); run_stage(8)
```

The builder requires the existing Drone body. It uses the installed S2 plant or adds S2 if all three of its subsystems are missing, builds the missing controller and mission stages, and connects the forward path and states bus. The S1 body and its hand-edited internal wiring remain the source of truth. The runner checks the parameters, runs test_stage1 through test_stage8, checks layout on the installed diagrams and exports the diagrams. Paste back the summary and any errors. For an interactive full mission, enter **52** in the stop-time toolbar and press **Run**. Waveforms contains the flight, motor and command scopes. Flight path shows the ground track and draws the 3-D path when the run ends.

| Stage | Python reference | MATLAB R2023a |
|---|---|---|
| S0 skeleton | Smoke test passed | User confirmed five passes after the empty-model fix |
| S1 body | Six cases and fixed-step ground recheck passed | Original five steps passed locally. Revised motors-off ground check awaits local run |
| S2 mixer and motors | Mixer signs, limits, slew, speed lag and torque passed | Written, not run |
| S3 rate | Frequency response, small steps and saturation recovery passed | Written, not run |
| S4 attitude | Angle steps passed. Heading drift exceeds the reporting threshold | Written, not run |
| S5 velocity and altitude | Four steps and tilt-compensation comparison passed | Written, not run |
| S6 mission | Timing, mirrored commands, yaw rotation and command integrals passed | Written, not run |
| S7 assembly | Closed-loop hover passed | Compilation, diagram geometry, scopes and export await local run |
| S8 complete system | Eight isolated maneuvers, mission, mass change and independent replay passed | Written, not run |

## Design and observations

**Rate control.** Each axis has a sampled PI with an integral bound and conditional integration at the output limit. Roll and pitch use the rotor thrust moment arm. For yaw, let D = max_speed²/(2 hover_speed), the local slope of rotor speed against normalized command. The four signed motor speed increments give shaft moment 4 D (2 kQ_prop hover_speed + J_total s)/(1 + motor_time_constant s) per yaw command. Divide by inertia_zz s + drag_rotational to obtain yaw rate. The inertia term opposes the roll-off from body inertia over the middle of the frequency range. A small yaw proportional gain of 0.05 leaves room for integral action at the 2 rad/s crossover. The sandbox measured roll/pitch crossover at 7.985 rad/s with 75.73° phase margin and yaw at 2.000 rad/s with 90.34°. These are hover linearizations, with nonlinear saturation tested separately. The isolated rate benches trim vertical thrust as the vehicle tilts so a sustained rate step stays airborne.

**Attitude.** Both 5° steps had a 0.704 s 10–90% rise time and zero measured overshoot. During the 3 s gap following the yaw-rate pulse, heading continued by 4.046°, above the 0.5° threshold. Yaw remains rate-controlled as specified. A proposed follow-up is to latch heading when the yaw command returns to zero and feed a bounded heading correction into the rate loop. That change requires approval.

**Velocity and altitude.** Horizontal PI bandwidth is 0.65 rad/s, below the inner attitude bandwidth. The PI zero matches the drag pole. Position hold integrates commanded climb and retains the target during hover. The climb feedforward weight is 1 − altitude_kp/climb_wc to reduce overshoot while the height reference moves. Measured overshoots were 0.25% horizontally, 5.00% in climb and 0.31% for the altitude step. A 2 m/s forward step lost 0.01756 m with tilt compensation off and 0.0000975 m with it on.

**Approved mission.** The table commands forward, back, right and left at 2 m/s for 4 s each, climb and descent at 1 m/s for 3 s each, and right and left yaw at 10°/s for 3 s each. Each maneuver has a 3 s hover gap. Current yaw rotates body forward/right into north/east references. The full mission reached a minimum height of 4.903 m and ended 0.005298 m from its start, within the 0.5 m limit.

**Isolated maneuvers.** Distance is measured at the end of the hover gap. The lag allowance is half a second of commanded travel. Final speed must be within 10% of the original command. Each horizontal maneuver covered 7.905 m versus 8 m commanded. Vertical travel was +3.038 and −3.038 m. Opposite horizontal and yaw commands exchanged the corresponding rotor traces to numerical precision. The climb/descent speed increments differed from perfect sign symmetry by 9.82% of peak increment, within the declared 15% bound. Thrust is quadratic in rotor speed.

**Mass change.** The 5 s hover held 5 m. At 300 kg the model used 2277.91 rpm, 51.2123 N m and 12.2163 kW per motor (48.8652 kW total). The 360 kg test starts in its corresponding hover trim. It used 2495.33 rpm and 61.4548 N m per motor. These results depend on the placeholder mass, inertia, drag, propeller and motor parameters.

## Reproduce the checks

In the project Python environment run

```sh
python -m pip install -r requirements.txt
python -m pip install -e .
pytest -q
python tests/py/recheck_stage8.py
```

The tests write expected/stage0.json through expected/stage8.json and expected/params.json. The independent recheck imports no dronesim code. It reconstructs the motor and controller equations from the design constants, evolves motor speed analytically under each held reference, and uses two midpoint steps per solver interval for the body. It compares each S8 metric and all four motor mirror measures. No disagreements exceeded the declared tolerances. The independently calculated mission return error was 0.0052984054 m.

Sandbox pytest excerpts:

```text
full_mission/position_error/end: [0, 0.5] | 0.00529840425 | PASS
Independent closed-loop replay: all S8 comparisons passed.
11 passed in 71.05s (0:01:11)
```

MATLAB source was syntax-checked with MISS_HIT. Simulink block compatibility, compilation, response comparisons, zero crossings and overlaps in the diagrams, and rendering require the local R2023a run.
