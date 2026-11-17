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


def ground_limit(depth):
    # During compression/rebound, m*z'' + (c+b)*z' + k*z = 0.
    # The unilateral normal force vanishes before z reaches zero.
    system = np.array([[0, 1], [-P.ground_stiffness/P.drone_mass,
                                 -(P.ground_damping+P.drag_linear)/P.drone_mass]])
    def contact(t):
        return expm(t*system) @ [depth, 0]

    def normal(t):
        down, velocity = contact(t)
        return P.ground_stiffness*down + P.ground_damping*velocity

    release = brentq(normal, 0, 2*np.sqrt(P.drone_mass/P.ground_stiffness))
    down, velocity = contact(release)
    # After release only linear drag acts. Integrate the exponential velocity.
    return -(down + velocity*P.drone_mass/P.drag_linear)


def test_body_cases():
    cases = {}
    hover = np.full(4, P.hover_speed)
    torque = P.kQ_prop * hover**2
    initial = P.initial_state
    zero = np.zeros_like(hover)
    thrust_delta = 10  # N per rotor, prescribed pitch test
    yaw_torque = 5  # N m per positive-spin rotor, prescribed yaw test
    depth = 0.5  # m below ground, prescribed contact test
    contact_state = initial.copy()
    contact_state[2] = depth
    differential = thrust_delta*np.array([-1, 1, -1, 1])
    pitch_speed = np.sqrt((P.drone_mass*P.gravity/4 + differential)/P.kT_prop)
    fall_time = 1.0
    drag_rate = P.drag_linear/P.drone_mass
    settle_time = 16/drag_rate
    scenarios = [
        ("hover", hover, torque, initial, P.drag_linear, fall_time),
        ("fall_free", zero, zero, initial, 0, fall_time),
        ("fall_drag", zero, zero, initial, P.drag_linear, fall_time),
        ("pitch", pitch_speed, torque, initial, P.drag_linear, fall_time),
        ("yaw", hover, np.where(P.motor_spin_dir > 0, yaw_torque, 0), initial,
         P.drag_linear, fall_time),
        ("ground", hover, torque, contact_state, P.drag_linear, settle_time),
    ]
    for name, speed, shaft, x0, drag, duration in scenarios:
        def rhs(t, state):
            return dynamics(speed, shaft, state, drag_linear=drag)

        trajectory = solve_ivp(rhs, (0, duration), x0, method="DOP853", rtol=1e-11, atol=1e-12)
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
            limit = ground_limit(depth)
            # After 16 drag time constants the remaining climb is below 0.1 mm.
            # Allow 1 cm between adaptive Python and fixed-step contact integration.
            compare("height_m", limit, -final[2], 1e-4, 0.01)
            compare("vertical_speed_mps", 0, final[5], 1e-6)
            compare("limiting_height_m", limit, -(final[2]+final[5]/drag_rate), 1e-6, 0.01)
            assert final[2] < 0
        cases[name] = {"speed": speed.tolist(), "torque": shaft.tolist(), "x0": x0.tolist(),
                       "drag": drag, "duration": duration, "checks": checks}

    fixture = {"states": STATE_NAMES, "cases": cases}
    (EXPECTED/"stage1.json").write_text(json.dumps(fixture, indent=2)+"\n", encoding="utf-8")


def test_ground_release_on_simulink_time_grid():
    # Check the contact transition at the configured ode4 step. Once airborne
    # under hover thrust, z + v_down/(drag/mass) is the exact limiting down.
    with (EXPECTED/"stage0.json").open(encoding="utf-8") as source:
        dt = json.load(source)["fixed_step_s"]["value"]
    depth = 0.5
    x = P.initial_state.copy()
    x[2] = depth
    hover = np.full(4, P.hover_speed)
    torque = P.kQ_prop*hover**2

    def f(state):
        return dynamics(hover, torque, state)

    for _ in range(round(1/dt)):
        k1 = f(x)
        k2 = f(x + dt*k1/2)
        k3 = f(x + dt*k2/2)
        k4 = f(x + dt*k3)
        x += (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
    assert x[2] < 0 and x[5] < 0
    got = -(x[2] + x[5]*P.drone_mass/P.drag_linear)
    target = ground_limit(depth)
    passed = abs(target-got) <= 0.01
    print(f"ground/fixed-step limiting height: {target} | {got} | {'PASS' if passed else 'FAIL'}")
    assert passed
