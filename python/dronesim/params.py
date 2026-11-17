"""SI parameters shared with matlab/drone_parameters.m."""
import numpy as _np

gravity = 9.81
drone_mass = 300
inertia_xx = 100
inertia_yy = 100
inertia_zz = 185
arm_length = 1.1 * _np.sqrt(2)
motor_x = arm_length / _np.sqrt(2) * _np.array([1, -1, 1, -1])
motor_y = arm_length / _np.sqrt(2) * _np.array([1, -1, -1, 1])
motor_spin_dir = _np.array([1, 1, -1, -1])
drag_linear = 20
drag_rotational = 50
ground_stiffness = 3e5
ground_damping = 2 * _np.sqrt(ground_stiffness * drone_mass)
ground_side_friction = 5e3
ground_level_stiffness = 2e4
ground_level_damping = 2e3
initial_state = _np.array([0, 0, -5, *([0] * 9)], dtype=float)
kT_prop = 0.01293
kQ_prop = 9e-4
J_prop = 0.7
tilt_compensation = 1
hover_speed = _np.sqrt(drone_mass * gravity / 4 / kT_prop)
