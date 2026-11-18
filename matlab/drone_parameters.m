%% 1. Mission table
% time [s], body forward/right/up [m/s], yaw rate [deg/s]. Approved mission.
mission_table = [0 2 0 0 0;
                 4 0 0 0 0;
                 7 -2 0 0 0;
                 11 0 0 0 0;
                 14 0 2 0 0;
                 18 0 0 0 0;
                 21 0 -2 0 0;
                 25 0 0 0 0;
                 28 0 0 1 0;
                 31 0 0 0 0;
                 34 0 0 -1 0;
                 37 0 0 0 0;
                 40 0 0 0 10;
                 43 0 0 0 0;
                 46 0 0 0 -10;
                 49 0 0 0 0;
                 52 0 0 0 0];  % SI except yaw rate, approved eight maneuvers
%% 2. Drone
gravity          = 9.81;          % m/s^2, gravitational acceleration
drone_mass       = 300;           % kg, total mass, PLACEHOLDER
inertia_xx       = 100;           % kg m^2, roll inertia, PLACEHOLDER
inertia_yy       = 100;           % kg m^2, pitch inertia, PLACEHOLDER
inertia_zz       = 185;           % kg m^2, yaw inertia, PLACEHOLDER
arm_length       = 1.1*sqrt(2);   % m, centre to each motor, PLACEHOLDER
motor_x          = arm_length/sqrt(2)*[ 1 -1  1 -1];  % m, forward position of motors 1..4 (FR, BL, FL, BR)
motor_y          = arm_length/sqrt(2)*[ 1 -1 -1  1];  % m, right position of motors 1..4
motor_spin_dir   = [1 1 -1 -1];   % +1: reaction torque turns the nose right
drag_linear      = 20;            % N/(m/s), body drag, PLACEHOLDER
drag_rotational  = 50;            % N m/(rad/s), body rotational drag, PLACEHOLDER
ground_stiffness       = 3e5;     % N/m, ground spring, PLACEHOLDER
ground_damping         = 2*sqrt(ground_stiffness*drone_mass);  % N s/m, critically damped
ground_side_friction   = 5e3;     % N s/m, sideways friction on the ground
ground_level_stiffness = 2e4;     % N m/rad, ground levels the body
ground_level_damping   = 2e3;     % N m s/rad
initial_state    = [0;0;-5; zeros(9,1)];  % drone starts airborne in hover at 5 m (NED, z down)

%% 3. Propeller
kT_prop = 0.01293;  % N/(rad/s)^2, thrust = kT*w^2
kQ_prop = 9e-4;     % N m/(rad/s)^2, drag torque = kQ*w^2, PLACEHOLDER
J_prop  = 0.7;      % kg m^2, propeller inertia, PLACEHOLDER

max_speed = 3000*pi/30;  % rad/s, 3000 rpm propeller limit, design brief

%% 4. Motor (ideal)
pole_pairs = 10;  % pole pairs, EMRAX 228
flux_linkage = 0.0542;  % V s, magnet flux, design brief
J_rotor = 0.0383;  % kg m^2, rotor inertia, EMRAX 228
I_max = 340/1.5;  % A peak, current limit, design brief
motor_time_constant = 0.0083;  % s, ideal speed lag, PLACEHOLDER
drive_sample_time = 0.00125;  % s, reference limiter updates once per ode4 step

%% 5. Flight controller
mixer_matrix = [-1 1 1; 1 -1 1; 1 1 -1; -1 -1 -1];  % dimensionless, roll/pitch/yaw to FR/BL/FL/BR
show_flight_path = 1;  % switch, draw the 3-D path at normal simulation stop
tilt_compensation = 1;  % 1 on, 0 off, specified tilt compensation switch
%% 6. Derived values
hover_speed = sqrt(drone_mass*gravity/4/kT_prop);  % rad/s, 4 props lift the weight
max_thrust = kT_prop*max_speed^2;  % N, full-command thrust per motor
hover_command = drone_mass*gravity/(4*max_thrust);  % fraction, four motors balance weight
torque_constant = 1.5*pole_pairs*flux_linkage;  % N m/A, amplitude-invariant dq
J_total = J_rotor + J_prop;  % kg m^2, rotor and propeller accelerate together
max_motor_torque = torque_constant*I_max;  % N m, shaft torque at the current limit
speed_slew_limit = 0.8*(max_motor_torque-kQ_prop*max_speed^2)/J_total;  % rad/s^2, leave 20% torque headroom after prop drag

rate_wc = [8;8;2];  % rad/s, roll/pitch/yaw rate design crossovers
rate_ratio = 0.25;  % dimensionless, roll/pitch PI zero divided by crossover
rate_plant_gain = 4*(arm_length/sqrt(2))*max_thrust./[inertia_xx;inertia_yy];  % rad/s^2 per axis demand
rate_mag = rate_plant_gain./(rate_wc(1:2).*sqrt(1+(motor_time_constant*rate_wc(1:2)).^2));  % rate magnitude at crossover
speed_per_command = max_speed^2/(2*hover_speed);  % rad/s per unit, local slope of sqrt thrust mapping
yaw_mag = abs(4*speed_per_command*(2*kQ_prop*hover_speed+1i*rate_wc(3)*J_total)/((drag_rotational+1i*rate_wc(3)*inertia_zz)*(1+1i*rate_wc(3)*motor_time_constant)));  % yaw-rate gain at crossover
yaw_kp = 0.05;  % demand per rad/s, small proportional path for inertia-dominated yaw
rate_kp = [1./(rate_mag*sqrt(1+rate_ratio^2));yaw_kp];  % demand per rad/s, unity crossover
rate_ki = [rate_kp(1:2).*rate_ratio.*rate_wc(1:2);rate_wc(3)*sqrt(1/yaw_mag^2-yaw_kp^2)];  % demand per rad, yaw PI magnitude includes Kp
axis_limit = [0.15;0.15;0.08];  % normalized, specified rate output limits
rate_i_limit = [0.10;0.10;0.08];  % normalized, specified rate integral limits
attitude_kp = rate_wc(1)/4;  % 1/s, attitude loop bandwidth below rate bandwidth
max_tilt_deg = 20;  % deg, tilt limit
max_tilt = max_tilt_deg*pi/180;  % rad, convert tilt limit for Euler angles
angle_rate_limit = 0.7;  % rad/s, attitude rate demand bound, design choice
velocity_wc = 0.65;  % rad/s, horizontal velocity bandwidth below attitude loop
climb_wc = 2.5;  % rad/s, climb velocity bandwidth
velocity_zero = drag_linear/drone_mass;  % 1/s, PI zero matches linear drag pole
velocity_kp = velocity_wc;  % 1/s, acceleration to velocity plant
velocity_ki = velocity_kp*velocity_zero;  % 1/s^2, cancel drag pole
climb_gain = 4*max_thrust/drone_mass;  % m/s^2 per collective, vertical acceleration gain
climb_mag = climb_gain/hypot(climb_wc,velocity_zero)/hypot(1,climb_wc*motor_time_constant);  % velocity gain at crossover
climb_kp = 1/(climb_mag*hypot(1,velocity_zero/climb_wc));  % collective per m/s, unity loop gain
climb_ki = climb_kp*velocity_zero;  % collective per m, drag zero
altitude_kp = climb_wc/4;  % 1/s, altitude loop bandwidth
accel_limit = gravity*max_tilt;  % m/s^2, horizontal PI output and integral bound
climb_limit = 0.35;  % fraction, symmetric climb PI and integral bound
