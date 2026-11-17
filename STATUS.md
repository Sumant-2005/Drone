# DroneSim_Simple handoff

| Stage | Python | MATLAB R2023a |
|---|---|---|
| S0 Skeleton | Smoke test passed | User reran S0 after the empty-model fix. All five steps passed. |
| S1 Drone body | All six scenarios passed, with an additional fixed-step contact check | Written, not run |
| S2–S8 | Pending | Pending |

## Run next

Pull the latest main branch. From the repository root in MATLAB R2023a with Simulink, run

```matlab
addpath('matlab'); run_stage(1)
```

Paste back the console output, including the comparison lines, the layout report and the five-step summary. The builder creates or replaces the Drone body subsystem in DroneSim.slx. The test requests a model update, then copies the body into a temporary model for constant speed and torque inputs. Each case gets its own initial state and drag setting. The saved diagram contains the body with its two inputs and one states bus output. A reshape adapts the integrated column to the Demux input, and a local Goto/From pair returns the integrated state to Dynamics.

## Python verification

From an activated virtual environment at the repository root, run

```sh
python -m pip install -r requirements.txt
python -m pip install -e .
python -c "import dronesim"
pytest -q
```

The suite writes the parameter reference and stage fixtures under expected/. The S1 tests check the continuous-time reference against independent calculations. The ground test also integrates the contact transition with RK4 on the configured 1.25 ms grid.

```text
fall_free/fall_m: 4.905 | 4.905000000000003 | PASS
fall_drag/fall_m: 4.797792711038241 | 4.797792711038252 | PASS
pitch/qdot: -0.44 | -0.44 | PASS
yaw/rdot: 0.05405405405405406 | 0.05405405405405406 | PASS
ground/height_m: 86.82118347477041 | 86.8211736414371 | PASS
3 passed in 0.66s
```

Hover kept all twelve states at their initial values. The fall distances are after one second. Pitch and yaw accelerations are the actual derivatives at the initial instant.

The prescribed ground test starts 0.5 m below ground at hover thrust. The spring pushes the body upward. Once the normal force reaches zero, thrust balances weight and linear drag slows the ascent. At 240 s the predicted height is 86.82117 m, with vertical speed below one micrometre per second. The independent contact solution gives a limiting height of 86.82118 m. The Python fixed-step recheck gives 86.82099 m. This height depends on the placeholder contact and drag parameters.

## Local results and open issues

The initial S0 run failed only when it attempted to simulate the empty model. After that call was removed, the user reported all five steps PASS in R2023a. S1 model compilation, simulation comparisons, layout counts and exported diagrams await the local run.
