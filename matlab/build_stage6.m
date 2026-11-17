function build_stage6()
s = flight_stage('Mission command',6,'Velocity and altitude control');
flight_add(s,'Simulation time','simulink/Sources/Clock',grid_position(1,1));
flight_add(s,'states','simulink/Ports & Subsystems/In1',grid_position(2,1));
flight_add(s,'Heading','simulink/Signal Routing/Bus Selector',grid_position(2,2),'OutputSignals','yaw');
code = strjoin({'function ref = mission(t,yaw)', ...
    'k = 1;', 'for j=2:size(mission_table,1)', ...
    '    if t >= mission_table(j,1), k=j; end', 'end', ...
    'row = mission_table(k,:);', ...
    'c = cos(yaw); s = sin(yaw);', ...
    'ref = [c*row(2)-s*row(3); s*row(2)+c*row(3); row(4); row(5)*pi/180; 0];', ...
    'end'},newline);
flight_function(s,'Read maneuver and rotate',grid_position(1,3),code,{'mission_table'});
flight_add(s,'command','simulink/Ports & Subsystems/Out1',grid_position(1,4));
flight_wire(s,'states/1','Heading/1','states');
flight_wire(s,'Simulation time/1','Read maneuver and rotate/1','Time (s)');
flight_wire(s,'Heading/1','Read maneuver and rotate/2','Yaw (rad)');
flight_wire(s,'Read maneuver and rotate/1','command/1','NED velocity, yaw rate, height offset (SI)');
save_system('DroneSim');
end
