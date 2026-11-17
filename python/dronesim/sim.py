"""Fixed-step integration of the open-loop stage-two plant."""
import numpy as np

from .body import dynamics
from .motors import motor_outputs


def rk4_step(rhs, state, dt):
    k1 = rhs(state)
    k2 = rhs(state + dt*k1/2)
    k3 = rhs(state + dt*k2/2)
    k4 = rhs(state + dt*k3)
    return state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)


def drive_derivatives(state, reference):
    """Twelve body states followed by the four motor speeds, with reference held."""
    speed = state[12:]
    dw, torque = motor_outputs(reference, speed)
    return np.concatenate((dynamics(speed, torque, state[:12]), dw))
