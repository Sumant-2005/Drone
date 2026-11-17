%% 1. Mission table
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
tilt_compensation = 1;  % 1 on, 0 off, specified tilt compensation switch
%% 6. Derived values
hover_speed = sqrt(drone_mass*gravity/4/kT_prop);  % rad/s, 4 props lift the weight
max_thrust = kT_prop*max_speed^2;  % N, full-command thrust per motor
hover_command = drone_mass*gravity/(4*max_thrust);  % fraction, four motors balance weight
torque_constant = 1.5*pole_pairs*flux_linkage;  % N m/A, amplitude-invariant dq
J_total = J_rotor + J_prop;  % kg m^2, rotor and propeller accelerate together
max_motor_torque = torque_constant*I_max;  % N m, shaft torque at the current limit
speed_slew_limit = 0.8*(max_motor_torque-kQ_prop*max_speed^2)/J_total;  % rad/s^2, leave 20% torque headroom after prop drag
