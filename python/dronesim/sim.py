"""Fixed-step integration of the open-loop stage-two plant."""
import numpy as np

from . import params as P
from .body import dynamics
from .motors import motor_outputs


def rk4_step(rhs, state, dt):
    k1 = rhs(state)
    k2 = rhs(state + dt*k1/2)
    k3 = rhs(state + dt*k2/2)
    k4 = rhs(state + dt*k3)
    return state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)


def drive_derivatives(state, reference, mass=P.drone_mass):
    """Twelve body states followed by the four motor speeds, with reference held."""
    speed = state[12:]
    dw, torque = motor_outputs(reference, speed)
    return np.concatenate((dynamics(speed, torque, state[:12], mass=mass), dw))


def flight(duration, *, mode="mission", value=None, table=None, mass=P.drone_mass, compensation=True,
           pulse_end=None, record_stride=4):
    """Run the complete nonlinear plant; inject demands at a loop input for step tests."""
    from .control import Controller
    from .mission import command
    from .mixer import mix
    from .motors import speed_reference

    dt = P.drive_sample_time
    ctrl = Controller(mass,compensation)
    ref = np.full(4,np.sqrt(mass*P.gravity/(4*P.kT_prop)))
    state = np.r_[P.initial_state,ref]
    n = round(duration/dt)
    rows = []
    for k in range(n+1):
        t = k*dt
        x = state[:12]
        u = command(t,x[8],table) if mode=="mission" else np.zeros(4)
        injected = value if pulse_end is None or t<pulse_end else np.zeros_like(value)
        if mode=="velocity":
            u = value
        angles,collective = ctrl.velocity(u,x,value if mode=="altitude" else 0.)
        if mode in ("rate","attitude"):
            angles = injected if mode=="attitude" else np.zeros(3)
            collective = ctrl.hover
        rates = injected if mode=="rate" else ctrl.attitude(angles,x)
        axis = ctrl.rate(rates,x)
        cmds = mix(collective,axis)
        ref = speed_reference(cmds,ref)
        if k%record_stride==0 or k==n:
            rows.append(np.r_[t,state,ref,motor_outputs(ref,state[12:])[1],u,angles,rates,axis,cmds,ctrl.rate_i,collective])
        if k<n:
            state = rk4_step(lambda y: drive_derivatives(y,ref,mass),state,dt)
    return np.asarray(rows)
