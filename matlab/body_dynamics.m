function xdot = body_dynamics(motor_speed, motor_torque, x)
% x = [north east down; v_north v_east v_down; roll pitch yaw; p q r]
v = x(4:6); phi = x(7); th = x(8); psi = x(9); w = x(10:12);
T = kT_prop * motor_speed(:).^2;
M = [-sum(motor_y(:).*T); sum(motor_x(:).*T); sum(motor_spin_dir(:).*motor_torque(:))];
cf = cos(phi); sf = sin(phi); ct = cos(th); st = sin(th); cp = cos(psi); sp = sin(psi);
R = [ct*cp, sf*st*cp - cf*sp, cf*st*cp + sf*sp;
     ct*sp, sf*st*sp + cf*cp, cf*st*sp - sf*cp;
     -st,   sf*ct,            cf*ct];
F = R*[0; 0; -sum(T)] + [0; 0; drone_mass*gravity] - drag_linear*v;
if x(3) > 0   % on or below the ground
    F(3)   = F(3) - max(0, ground_stiffness*x(3) + ground_damping*v(3));
    F(1:2) = F(1:2) - ground_side_friction*v(1:2);
    M = M - [ground_level_stiffness*phi + ground_level_damping*w(1);
             ground_level_stiffness*th  + ground_level_damping*w(2);
             ground_level_damping*w(3)];
end
I = diag([inertia_xx inertia_yy inertia_zz]);
wdot = I \ (M - cross(w, I*w) - drag_rotational*w);
eulDot = [w(1) + sf*st/ct*w(2) + cf*st/ct*w(3);
          cf*w(2) - sf*w(3);
          (sf*w(2) + cf*w(3))/ct];
xdot = [v; F/drone_mass; eulDot; wdot];
end
