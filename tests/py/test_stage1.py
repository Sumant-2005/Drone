"""Independent analytic checks and common fixtures for the Simulink body test."""
import json
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from scipy.optimize import brentq

from dronesim import params as P
from dronesim.body import STATE_NAMES, dynamics

EXPECTED = Path(__file__).resolve().parents[2] / "expected"


def test_body_cases():
    cases = {}
    hover = np.full(4, P.hover_speed)
    torque = P.kQ_prop * hover**2
    initial = P.initial_state
    zero = np.zeros_like(hover)
    thrust_delta = 10  # N per rotor, prescribed pitch test
    yaw_torque = 5  # N m per positive-spin rotor, prescribed yaw test
    contact_state = np.zeros_like(initial)
    differential = thrust_delta*np.array([-1, 1, -1, 1])
    pitch_speed = np.sqrt((P.drone_mass*P.gravity/4 + differential)/P.kT_prop)
    fall_time = 1.0
    drag_rate = P.drag_linear/P.drone_mass
    step = json.loads((EXPECTED/"stage0.json").read_text())["fixed_step_s"]["value"]
    band = 0.02
    deadline = 0.5
    scenarios = [
        ("hover", hover, torque, initial, P.drag_linear, fall_time),
        ("fall_free", zero, zero, initial, 0, fall_time),
        ("fall_drag", zero, zero, initial, P.drag_linear, fall_time),
        ("pitch", pitch_speed, torque, initial, P.drag_linear, fall_time),
        ("yaw", hover, np.where(P.motor_spin_dir > 0, yaw_torque, 0), initial,
         P.drag_linear, fall_time),
        ("ground", zero, zero, contact_state, P.drag_linear, fall_time),
    ]
    for name, speed, shaft, x0, drag, duration in scenarios:
        def rhs(t, state):
            return dynamics(speed, shaft, state, drag_linear=drag)

        grid = np.linspace(0, duration, round(duration/step)+1) if name == "ground" else None
        trajectory = solve_ivp(rhs, (0, duration), x0, method="DOP853", t_eval=grid, rtol=1e-11, atol=1e-12)
        assert trajectory.success
        dx = rhs(0, x0)
        history = trajectory.y
        final = history[:, -1]
        checks = {}

        def compare(key, target, got, atol, matlab_atol=None):
            passed = np.allclose(got, target, rtol=0, atol=atol)
            print(f"{name}/{key}: {target} | {got} | {'PASS' if passed else 'FAIL'}")
            assert passed
            checks[key] = {"value": np.asarray(got).tolist(),
                           "atol": atol if matlab_atol is None else matlab_atol}

        if name == "hover":
            compare("max_state_error", np.zeros_like(x0),
                    np.max(np.abs(history-x0[:, None]), axis=1), 1e-10)
        elif name.startswith("fall"):
            fall = P.gravity*duration**2/2 if drag == 0 else (
                P.gravity/drag_rate*(duration+np.expm1(-drag_rate*duration)/drag_rate))
            compare("fall_m", fall, final[2]-x0[2], 1e-9)
        elif name == "pitch":
            moment = -4*(P.arm_length/np.sqrt(2))*thrust_delta
            compare("moment_Nm", moment, P.inertia_yy*dx[10], 1e-10)
            compare("qdot", moment/P.inertia_yy, dx[10], 1e-11)
            compare("initial_down_accel", 0, dx[5], 1e-11)
        elif name == "yaw":
            compare("rdot", 2*yaw_torque/P.inertia_zz, dx[11], 1e-11)
            compare("roll_pitch_rate_peak", 0, np.max(np.abs(history[[6, 7, 9, 10]])), 1e-10)
        else:
            equilibrium = P.drone_mass*P.gravity/P.ground_stiffness
            compare("down_m", equilibrium, final[2], 1e-10)
            compare("overshoot_m", 0, max(0, history[2].max()-equilibrium), 1e-11)
            compare("vertical_speed_mps", 0, final[5], 1e-9)
            outside = np.flatnonzero(abs(history[2]-equilibrium) > band*equilibrium)
            settling = grid[outside[-1]+1]
            # Independent step response of the loaded spring with both dampers.
            A = [[0, 1], [-P.ground_stiffness/P.drone_mass,
                          -(P.ground_damping+P.drag_linear)/P.drone_mass]]
            def excess(t):
                return expm(np.asarray(A)*t)[0, 0]-band
            exact = brentq(excess, 0, deadline)
            compare("settling_s", exact, settling, step)
            assert settling <= deadline
        cases[name] = {"speed": speed.tolist(), "torque": shaft.tolist(), "x0": x0.tolist(),
                       "drag": drag, "duration": duration, "checks": checks}

    cases["ground"].update(band_fraction=band, deadline_s=deadline)
    fixture = {"states": STATE_NAMES, "cases": cases}
    (EXPECTED/"stage1.json").write_text(json.dumps(fixture, indent=2)+"\n", encoding="utf-8")


def test_ground_on_fixed_grid():
    from dronesim.sim import rk4_step

    ground = np.zeros_like(P.initial_state)
    zero = np.zeros(4)
    dt = json.loads((EXPECTED/"stage0.json").read_text())["fixed_step_s"]["value"]
    equilibrium = P.drone_mass*P.gravity/P.ground_stiffness
    history = [ground[2]]
    for _ in range(round(1/dt)):
        ground = rk4_step(lambda x: dynamics(zero, zero, x), ground, dt)
        history.append(ground[2])
    outside = np.flatnonzero(abs(np.asarray(history)-equilibrium) > 0.02*equilibrium)
    settling = (outside[-1]+1)*dt
    passed = (settling <= 0.5 and max(history) <= equilibrium+1e-11
              and abs(ground[2]-equilibrium) <= 1e-10)
    print(f"ground/fixed-step settling_s: <=0.5 | {settling} | {'PASS' if passed else 'FAIL'}")
    assert passed
