"""Generate inputs and numerical references for the four Simulink test benches."""
import json
from pathlib import Path

import numpy as np

from dronesim import params as P
from dronesim.mixer import mix
from dronesim.motors import motor_outputs, speed_reference
from dronesim.sim import drive_derivatives, rk4_step

EXPECTED = Path(__file__).resolve().parents[2] / "expected"
DT = P.drive_sample_time
HOVER = np.full(4, P.hover_speed)
STEP_TIME = 8*DT  # Start in hover before applying each test input.


def record(checks, label, expected, got, atol, cross_solver_atol=None):
    passed = np.allclose(got, expected, atol=atol, rtol=0)
    print(f"{label}: {expected} | {got} | {'PASS' if passed else 'FAIL'}")
    assert passed
    checks[label] = {"value": np.asarray(got).tolist(),
                     "atol": atol if cross_solver_atol is None else cross_solver_atol}


def test_motor_limits():
    # Independent checks in the original engineering units and torque budget.
    for label, target, got, atol in [
        ("maximum rpm", 3000, P.max_speed*60/(2*np.pi), 1e-9),
        ("hover thrust (N)", P.drone_mass*P.gravity/4, P.hover_command*P.max_thrust, 1e-9),
        ("maximum thrust (N)", 1276, P.max_thrust, 1),
        ("torque constant (N m/A)", 0.813, P.torque_constant, 1e-12),
        ("shaft torque limit (N m)", 184.3, P.max_motor_torque, 0.05),
        ("accelerating inertia (kg m^2)", 0.7383, P.J_total, 1e-12),
        ("torque headroom fraction", 0.2,
         1-P.J_total*P.speed_slew_limit/(P.max_motor_torque-P.kQ_prop*P.max_speed**2), 1e-12),
    ]:
        record({}, label, target, got, atol)
    assert P.kQ_prop*P.max_speed**2 < 96  # N m continuous rating from the brief


def test_stage2():
    dt = json.loads((EXPECTED/"stage0.json").read_text())["fixed_step_s"]["value"]
    assert DT == dt
    cases = {}

    def case(name, kind, duration, **inputs):
        checks = {}
        cases[name] = dict(kind=kind, duration=duration, checks=checks, **inputs)
        return checks

    # One channel per clamp probe. Hold long enough for a full-speed traverse.
    hold = np.ceil((P.max_speed/P.speed_slew_limit + 1)/DT)*DT
    for name, cmd in (("hover_ref", np.full(4, P.hover_command)),
                      ("rounded_hover_ref", np.full(4, 0.5766)),
                      ("limits", np.array([0., 1., 1.2, -0.2]))):
        duration = STEP_TIME+hold
        checks = case(name, "thrust", duration, command=cmd.tolist())
        prev = HOVER.copy()
        peak_rate = np.zeros(4)
        for k in range(round(duration/DT)+1):
            next_ref = speed_reference(P.hover_command*np.ones(4) if k < round(STEP_TIME/DT) else cmd, prev)
            peak_rate = np.maximum(peak_rate, abs(next_ref-prev)/DT)
            prev = next_ref
        target = P.max_speed*np.sqrt(np.clip(cmd, 0, 1))
        record(checks, "final_ref", target, prev, 1e-9)
        expected_rate = np.minimum(abs(target-HOVER)/DT, P.speed_slew_limit)
        record(checks, "peak_slew", expected_rate, peak_rate, 1e-8)

    demand = 0.1
    patterns = [[-1, 1, 1, -1], [1, -1, 1, -1], [1, 1, -1, -1]]
    for axis, pattern in enumerate(patterns):
        axis_input = demand*np.eye(3)[axis]
        for collective in ([P.hover_command, 0.95] if axis == 0 else [P.hover_command]):
            name = "clip" if collective == 0.95 else ("roll_mix", "pitch_mix", "yaw_mix")[axis]
            checks = case(name, "mixer", STEP_TIME, collective=collective, axis=axis_input.tolist())
            cmd = mix(collective, axis_input)
            expected = np.clip(collective+demand*np.asarray(pattern), 0, 1)
            record(checks, "command", expected, cmd, 1e-12)
            record(checks, "command_sum", sum(expected), sum(cmd), 1e-12)

    step_size = 2.0  # rad/s, specified bypass step
    duration = STEP_TIME+np.ceil(8*P.motor_time_constant/DT)*DT
    for name, delta in (("motor_hover", 0.), ("motor_step", step_size)):
        checks = case(name, "motor", duration, delta=delta)
        times = np.arange(round(duration/DT)+1)*DT
        speed = HOVER.copy()
        history, torques = [], []
        for k, t in enumerate(times):
            ref = HOVER + (delta if k >= round(STEP_TIME/DT) else 0)
            history.append(speed)
            torques.append(motor_outputs(ref, speed)[1])
            speed = rk4_step(lambda w: motor_outputs(ref, w)[0], speed, DT)
        history, torques = np.asarray(history), np.asarray(torques)
        if delta == 0:
            record(checks, "speed_error", 0, np.max(abs(history-HOVER)), 1e-10)
            record(checks, "torque", P.kQ_prop*HOVER**2, torques[-1], 1e-9)
            record(checks, "torque_drift", 0, np.max(abs(torques-torques[0])), 1e-9)
        else:
            fraction = (history[:, 0]-P.hover_speed)/delta
            crossing = np.interp(1-np.exp(-1), fraction, times)-STEP_TIME
            record(checks, "tau_s", P.motor_time_constant, crossing, DT/10)
            record(checks, "fraction_at_tau", 1-np.exp(-1),
                   np.interp(STEP_TIME+P.motor_time_constant, times, fraction), 0.002)
            record(checks, "torque_at_step", P.kQ_prop*HOVER**2+P.J_total*delta/P.motor_time_constant,
                   torques[round(STEP_TIME/DT)], 1e-7)

    for axis, name in enumerate(("roll", "pitch", "yaw")):
        axis_input = 0.02*np.eye(3)[axis]
        duration = STEP_TIME+0.2
        checks = case(name, "chain", duration, collective=P.hover_command,
                      axis=axis_input.tolist(), axis_number=axis+1)
        state = np.r_[P.initial_state, HOVER]
        ref = HOVER.copy()
        for k in range(round(duration/DT)+1):
            cmd = mix(P.hover_command, axis_input if k >= round(STEP_TIME/DT) else np.zeros(3))
            ref = speed_reference(cmd, ref)
            rhs = lambda x: drive_derivatives(x, ref)
            if k < round(duration/DT):
                state = rk4_step(rhs, state, DT)
        acceleration = rhs(state)[9+axis]
        record(checks, "positive_accel", 1, int(acceleration > 0), 0)
        print(f"{name} acceleration reference: {acceleration} rad/s^2")
        checks["axis_accel"] = {"value": float(acceleration), "atol": 2e-4}

    fixture = dict(step_time=STEP_TIME, cases=cases)
    (EXPECTED/"stage2.json").write_text(json.dumps(fixture, indent=2)+"\n")
