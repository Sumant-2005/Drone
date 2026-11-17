# DroneSim_Simple handoff

| Stage | Python | MATLAB R2023a |
|---|---|---|
| S0 Skeleton | Smoke test passed | All five steps passed on the user's rerun. |
| S1 Drone body | Six body cases and a fixed-step ground check passed | User reported all five steps PASS on the original S1. The revised ground case awaits the combined run. |
| S2 Mixer, thrust to speed, ideal motors | Parameter, component and connected-body tests passed | Written, not run. |
| S3–S8 | Pending | Pending |

## Run next

Pull main. From the repository root containing your saved DroneSim.slx, run

```matlab
addpath('matlab'); run_stage(2)
```

Paste the comparison lines, layout report and six-step summary. The order is build_stage2, test_params, test_stage1, test_stage2, check_layout, export_diagrams.

Your local DroneSim.slx is the source of truth for the finished body. The S2 builder requires DroneSim/Drone body and checks for it before editing the model. It places its three subsystems upstream and connects speed and torque to the existing body ports. On a rebuild it replaces its tagged S2 subsystems. An occupied input or an unrelated wire on an S2 subsystem is reported before replacement. The body tests use a temporary copy of the installed body. S2 checks the four connections on the installed model and exercises copies of the installed blocks in test benches.

For an explicit fresh S1 build, build_stage1 aligns the diagram to the actual port centers and branches vertically to the feedback tag. The S2 runner uses the saved body directly.

## Python verification

With the repository virtual environment active, run `pytest -q`. The tests generate expected/params.json and the stage comparison fixtures. Excerpts from the sandbox run follow.

```text
ground/down_m: 0.00981 | 0.009809999999992026 | PASS
ground/overshoot_m: 0 | 0 | PASS
ground/settling_s: 0.18486283531640899 | 0.185 | PASS
tau_s: 0.0083 | 0.008321564354160328 | PASS
5 passed in 1.02s
```

The corrected ground test starts at the surface at rest with zero speed and shaft torque on all four motors. Gravity loads the spring until it supports the weight. Settling time is the first sample after which penetration stays within 2% of mass times gravity divided by stiffness, observed through one second. Both the continuous reference sampled on the solver grid and the fixed-step RK4 recheck meet the 0.5 s deadline. The S1 body equations are unchanged.

At exact hover the reference was 238.54256 rad/s and shaft torque was 51.21230 N m per motor. The rounded command 0.5766 was tested separately. The clipped command probes reached 0 or 314.15927 rad/s and the measured peak slew was 103.43065 rad/s². A sample-and-hold on the 1.25 ms solver grid lets the inherited Rate Limiter use its hover initial condition. The fixtures hold the command long enough to complete a full speed traverse. The isolated motor time constant check bypasses the limiter and interpolates between solver samples.

Roll, pitch and yaw mixer patterns and saturation passed. With a positive 0.02 demand on one axis at a time, the connected Python plant gave positive accelerations 0.2 s after the step on all three axes. The slew rate, torque and transient responses depend on the specified propeller and ideal motor parameters.

## Local verification pending

The initial S0 empty-model simulation failure was resolved and S0 passed locally. The user also confirmed the original S1 model passed in R2023a and manually straightened its wires. The corrected S1 ground comparison, fresh-build S1 layout code and S2 scripts are written, not run in MATLAB here. The fresh S1 layout code is exercised only by an explicit S1 rebuild.
