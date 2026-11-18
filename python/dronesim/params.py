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

max_speed = 3000 * _np.pi / 30
pole_pairs = 10
flux_linkage = 0.0542
J_rotor = 0.0383
I_max = 340 / 1.5
motor_time_constant = 0.0083
drive_sample_time = 0.00125
mixer_matrix = _np.array([[-1, 1, 1], [1, -1, 1], [1, 1, -1], [-1, -1, -1]])
max_thrust = kT_prop * max_speed**2
hover_command = drone_mass * gravity / (4 * max_thrust)
torque_constant = 1.5 * pole_pairs * flux_linkage
J_total = J_rotor + J_prop
max_motor_torque = torque_constant * I_max
speed_slew_limit = 0.8 * (max_motor_torque - kQ_prop * max_speed**2) / J_total

# Approved maneuvers in body forward/right/up and degrees/s.
mission_table = _np.array([[0,2,0,0,0],[4,0,0,0,0],[7,-2,0,0,0],[11,0,0,0,0],[14,0,2,0,0],[18,0,0,0,0],[21,0,-2,0,0],[25,0,0,0,0],[28,0,0,1,0],[31,0,0,0,0],[34,0,0,-1,0],[37,0,0,0,0],[40,0,0,0,10],[43,0,0,0,0],[46,0,0,0,-10],[49,0,0,0,0],[52,0,0,0,0]], dtype=float)
rate_wc = _np.array([8., 8., 2.])
rate_ratio = 0.25
rate_plant_gain = 4*(arm_length/_np.sqrt(2))*max_thrust/_np.array([inertia_xx, inertia_yy])
rate_mag = rate_plant_gain/(rate_wc[:2]*_np.sqrt(1+(motor_time_constant*rate_wc[:2])**2))
speed_per_command = max_speed**2/(2*hover_speed)
yaw_mag = abs(4*speed_per_command*(2*kQ_prop*hover_speed+1j*rate_wc[2]*J_total) /
              ((drag_rotational+1j*rate_wc[2]*inertia_zz)*(1+1j*rate_wc[2]*motor_time_constant)))
yaw_kp = 0.05
rate_kp = _np.r_[1/(rate_mag*_np.sqrt(1+rate_ratio**2)), yaw_kp]
rate_ki = _np.r_[rate_kp[:2]*rate_ratio*rate_wc[:2], rate_wc[2]*_np.sqrt(1/yaw_mag**2-yaw_kp**2)]
axis_limit = _np.array([0.15, 0.15, 0.08])
rate_i_limit = _np.array([0.10, 0.10, 0.08])
attitude_kp = rate_wc[0]/4
max_tilt_deg = 20
max_tilt = _np.deg2rad(max_tilt_deg)
angle_rate_limit = 0.7
velocity_wc = 0.65
climb_wc = 2.5
velocity_zero = drag_linear/drone_mass
velocity_kp = velocity_wc
velocity_ki = velocity_kp*velocity_zero
climb_gain = 4*max_thrust/drone_mass
climb_mag = climb_gain/_np.hypot(climb_wc,velocity_zero)/_np.hypot(1,climb_wc*motor_time_constant)
climb_kp = 1/(climb_mag*_np.hypot(1,velocity_zero/climb_wc))
climb_ki = climb_kp*velocity_zero
altitude_kp = climb_wc/4
accel_limit = gravity*max_tilt
climb_limit = 0.35

show_flight_path = 1
