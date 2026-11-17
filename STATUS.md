# DroneSim_Simple handoff

| Stage | Python | MATLAB R2023a |
|---|---|---|
| S0 Skeleton | Smoke test passed | Written, not run |
| S1–S8 | Pending | Pending |

## Run next

Open MATLAB R2023a with Simulink, set the current folder to this repository root and run the following line.

```matlab
addpath('matlab'); run_stage(0)
```

The runner creates DroneSim.slx, checks the parameters and solver settings, simulates the empty model, checks the diagram layout and exports PNGs into results/diagrams/. Paste the console output, including the summary table and any error details, into the next task.

## Python verification

Using an activated Python virtual environment at the repository root, run these commands to reproduce the smoke test and JSON references.

```sh
python -m pip install -r requirements.txt
python -m pip install -e .
python -c "import dronesim"
pytest -q
```

The import succeeded. The smoke test imported the six stage modules, checked the S0 parameter set and verified the one-second reference grid using the specified 1.25 ms step. The expected JSON files record the reference values for the MATLAB tests.

```text
1 passed in 0.01s
```

## Local results and open issues

Awaiting the first MATLAB run. In particular, only that run can confirm R2023a accepts the callbacks and solver, simulates the empty diagram, and exports it. The Python modules for future stages are empty skeleton files. No flight dynamics have been implemented.
