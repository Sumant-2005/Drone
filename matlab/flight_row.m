function flight_row(parent,name,place,signal,kind,kp,ki,limit,ilimit)
% Each feedback row reads its own scalar field from the state bus.
s = flight_add(parent,name,'built-in/Subsystem',place);
flight_add(s,'Demand','simulink/Ports & Subsystems/In1',grid_position(1,1),'Port','1');
flight_add(s,'states','simulink/Ports & Subsystems/In1',grid_position(2,1),'Port','2');
flight_add(s,'Measurement','simulink/Signal Routing/Bus Selector',grid_position(2,2),'OutputSignals',signal);
flight_add(s,'Error','simulink/Math Operations/Sum',grid_position(1,3),'Inputs','+-','IconShape','rectangular');
flight_wire(s,'states/1','Measurement/1','states');
flight_wire(s,'Demand/1','Error/1','Reference (SI)');
if strcmp(signal,'v_down')
    flight_add(s,'Climb speed','simulink/Math Operations/Gain',grid_position(2,3),'Gain','-1');
    flight_wire(s,'Measurement/1','Climb speed/1','Down speed (m/s)');
    flight_wire(s,'Climb speed/1','Error/2','Climb speed (m/s)');
else
    flight_wire(s,'Measurement/1','Error/2','Measured state (SI)');
end
if strcmp(kind,'P')
    flight_add(s,'Angle gain','simulink/Math Operations/Gain',grid_position(1,4),'Gain',kp);
    flight_add(s,'Rate limit','simulink/Discontinuities/Saturation',grid_position(1,5), ...
        'UpperLimit',limit,'LowerLimit',['-(' limit ')']);
    flight_wire(s,'Error/1','Angle gain/1','Angle error (rad)');
    flight_wire(s,'Angle gain/1','Rate limit/1','Requested rate (rad/s)');
    final = 'Rate limit/1';
else
    flight_add(s,'Sample error','simulink/Discrete/Zero-Order Hold',grid_position(1,4),'SampleTime','drive_sample_time');
    code = strjoin({'function [u,integral] = pi_step(e)', ...
        'persistent memory', 'if isempty(memory), memory=0; end', ...
        ['raw = (' kp ')*e + memory;'], ...
        ['u = min(max(raw,-(' limit ')),(' limit '));'], ...
        'if (raw-u)*e <= 0', ...
        ['    memory = min(max(memory+drive_sample_time*(' ki ')*e,-(' ilimit ')),(' ilimit '));'], ...
        'end','integral = memory;','end'},newline);
    pars = unique(regexp(strjoin({kp,ki,limit,ilimit,'drive_sample_time'}),'[A-Za-z_]+','match'));
    flight_function(s,'Clamped PI',grid_position(1,5),code,pars);
    flight_add(s,'Integral monitor','simulink/Sinks/Terminator',grid_position(2,6));
    flight_wire(s,'Error/1','Sample error/1','Control error (SI)');
    flight_wire(s,'Sample error/1','Clamped PI/1','Sampled error (SI)');
    flight_wire(s,'Clamped PI/2','Integral monitor/1','Integral contribution (SI)');
    final = 'Clamped PI/1';
end
flight_add(s,'Output','simulink/Ports & Subsystems/Out1',grid_position(1,6));
flight_wire(s,final,'Output/1','Limited demand (SI)');
end
