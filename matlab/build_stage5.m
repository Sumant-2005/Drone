function build_stage5()
s = flight_stage('Velocity and altitude control',5,'Attitude control');
controller_ports(s,5);
for i=1:2
    names = {'North','East'};
    fields = {'v_north','v_east'};
    name = names{i};
    flight_row(s,name,grid_position(2*i-1,5),fields{i},'PI','velocity_kp','velocity_ki','accel_limit','accel_limit');
    flight_wire(s,sprintf('Split references/%d',i),[name '/1'],'Velocity reference (m/s)');
    controller_feedback(s,[name ' feedback'],2*i-0.4,4,[name '/2']);
end
a = flight_add(s,'Altitude reference','built-in/Subsystem',grid_position(5,5));
for i=1:3
    names={'Climb','Offset','states'};
    flight_add(a,names{i},'simulink/Ports & Subsystems/In1',grid_position(i,1),'Port',num2str(i));
end
flight_add(a,'Height demand','simulink/Discrete/Discrete-Time Integrator',grid_position(1,2), ...
    'InitialCondition','-initial_state(3)','SampleTime','drive_sample_time');
flight_add(a,'Measured down','simulink/Signal Routing/Bus Selector',grid_position(3,2),'OutputSignals','down');
code = sprintf(['function v = outer(height,offset,down,climb)\n' ...
    'v = (1-altitude_kp/climb_wc)*climb+altitude_kp*(height+offset+down);\nend']);
flight_function(a,'Altitude hold',grid_position(2,4),code,{'altitude_kp','climb_wc'});
flight_add(a,'Velocity demand','simulink/Ports & Subsystems/Out1',grid_position(2,5));
flight_wire(a,'Climb/1','Height demand/1','Climb (m/s)');
flight_wire(a,'Height demand/1','Altitude hold/1','Height demand (m)');
flight_wire(a,'Offset/1','Altitude hold/2','Height step (m)');
flight_wire(a,'states/1','Measured down/1','states');
flight_wire(a,'Measured down/1','Altitude hold/3','Down (m)');
flight_wire(a,'Climb/1','Altitude hold/4');
flight_wire(a,'Altitude hold/1','Velocity demand/1','Climb demand (m/s)');
flight_wire(s,'Split references/3','Altitude reference/1','Climb (m/s)');
flight_wire(s,'Split references/5','Altitude reference/2','Height offset (m)');
controller_feedback(s,'Height feedback',6,4,'Altitude reference/3');
flight_row(s,'Climb',grid_position(5,7),'v_down','PI','climb_kp','climb_ki','climb_limit','climb_limit');
flight_wire(s,'Altitude reference/1','Climb/1','Climb demand (m/s)');
controller_feedback(s,'Climb feedback',6,6,'Climb/2');
m = flight_add(s,'Convert demands','built-in/Subsystem',grid_position(3,9));
names={'North','East','Yaw rate','Increment','states'};
for i=1:5
    flight_add(m,names{i},'simulink/Ports & Subsystems/In1',grid_position(i,1),'Port',num2str(i));
end
flight_add(m,'Angles','simulink/Signal Routing/Bus Selector',grid_position(5,2),'OutputSignals','roll,pitch,yaw');
flight_wire(m,'states/1','Angles/1','states');
code=strjoin({'function [d,u] = convert(n,e,r,inc,phi,theta,yaw)', ...
    'c=cos(yaw); s=sin(yaw);', ...
    'tilt=[-s*n+c*e; -(c*n+s*e)]/gravity;', ...
    'd=[tilt/max(1,norm(tilt)/max_tilt); r];', ...
    'u=hover_command+inc;', ...
    'if tilt_compensation, u=u/max(cos(phi)*cos(theta),cos(max_tilt)^2); end', ...
    'u=min(1,max(0,u));','end'},newline);
flight_function(m,'Body axes and collective',grid_position(3,4),code,{'gravity','max_tilt','hover_command','tilt_compensation'});
for i=1:4
    flight_wire(m,[names{i} '/1'],sprintf('Body axes and collective/%d',i),'Demand (SI)');
end
for i=1:3
    flight_wire(m,sprintf('Angles/%d',i),sprintf('Body axes and collective/%d',i+4),'Attitude (rad)');
end
for i=1:2
    names={'Attitude demand','Collective'};
    flight_add(m,names{i},'simulink/Ports & Subsystems/Out1',grid_position(i+2,5),'Port',num2str(i));
    flight_wire(m,sprintf('Body axes and collective/%d',i),[names{i} '/1'],'Demand (SI)');
    flight_add(s,names{i},'simulink/Ports & Subsystems/Out1',grid_position(i+2,10),'Port',num2str(i));
    flight_wire(s,sprintf('Convert demands/%d',i),[names{i} '/1'],'Demand (SI)');
end
sources = {'North/1','East/1','Split references/4','Climb/1'};
for i=1:4
    flight_wire(s,sources{i},sprintf('Convert demands/%d',i),'Demand (SI)');
end
controller_feedback(s,'Rotation feedback',4,8,'Convert demands/5');
save_system('DroneSim');
end
