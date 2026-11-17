"""Rigid-body dynamics in north/east/down coordinates. All inputs use SI."""
import numpy as np
from . import params as P

STATE_NAMES = ("north", "east", "down", "v_north", "v_east", "v_down",
               "roll", "pitch", "yaw", "p", "q", "r")


def dynamics(motor_speed, motor_torque, x, *, drag_linear=None):
    x = np.asarray(x).reshape(12)
    v, angles, w = x[3:6], x[6:9], x[9:12]
    thrust = P.kT_prop * np.asarray(motor_speed).reshape(4)**2
    torque = np.asarray(motor_torque).reshape(4)
    moment = np.array([-np.sum(P.motor_y*thrust), np.sum(P.motor_x*thrust),
                       np.sum(P.motor_spin_dir*torque)])
    cf, ct, cp = np.cos(angles)
    sf, st, sp = np.sin(angles)
    rotation = np.array([[ct*cp, sf*st*cp-cf*sp, cf*st*cp+sf*sp],
                         [ct*sp, sf*st*sp+cf*cp, cf*st*sp-sf*cp],
                         [-st, sf*ct, cf*ct]])
    drag = P.drag_linear if drag_linear is None else drag_linear
    force = rotation @ [0, 0, -sum(thrust)] + [0, 0, P.drone_mass*P.gravity] - drag*v
    if x[2] > 0:
        force[2] -= max(0, P.ground_stiffness*x[2] + P.ground_damping*v[2])
        force[:2] -= P.ground_side_friction*v[:2]
        moment -= P.ground_level_damping*w
        moment[:2] -= P.ground_level_stiffness*angles[:2]
    inertia = np.array([P.inertia_xx, P.inertia_yy, P.inertia_zz])
    wdot = (moment - np.cross(w, inertia*w) - P.drag_rotational*w) / inertia
    euler_dot = [w[0]+sf*st/ct*w[1]+cf*st/ct*w[2],
                 cf*w[1]-sf*w[2], (sf*w[1]+cf*w[2])/ct]
    return np.concatenate((v, force/P.drone_mass, euler_dot, wdot))
