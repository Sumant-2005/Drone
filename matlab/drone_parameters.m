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

%% 4. Motor (ideal)
%% 5. Flight controller
tilt_compensation = 1;  % 1 on, 0 off, specified tilt compensation switch
%% 6. Derived values
hover_speed = sqrt(drone_mass*gravity/4/kT_prop);  % rad/s, 4 props lift the weight
