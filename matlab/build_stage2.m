function build_stage2()
% The local SLX owns the finished body, including its hand-edited layout.
mdl = 'DroneSim';
body = [mdl '/Drone body'];
assert(isfile([mdl '.slx']), 'Missing DroneSim.slx. Open the saved S1 model containing Drone body.');
[~,saved] = fileattrib([mdl '.slx']);
load_system(saved.Name);
assert(getSimulinkBlockHandle(body) ~= -1 && strcmp(get_param(body,'BlockType'),'SubSystem'), ...
    'Missing DroneSim/Drone body in the saved model. Restore your finished S1 model before running S2.');
names = {'Mixer','Thrust to speed','Motors (ideal, placeholder)'};
paths = strcat([mdl '/'],names);
owner = 'DroneSim_S2';
% Validate every name and endpoint before replacing any S2 diagrams.
for k = 1:numel(paths)
    if getSimulinkBlockHandle(paths{k}) ~= -1
        assert(strcmp(get_param(paths{k},'Tag'),owner),'Existing block %s is not owned by S2.',paths{k});
    end
end
links = {paths{1},1,paths{2},1; paths{2},1,paths{3},1; ...
    paths{3},1,body,1; paths{3},2,body,2};
% Reject unrelated external wires rather than disconnecting them on a rebuild.
for k = 1:numel(paths)
    if getSimulinkBlockHandle(paths{k}) == -1
        continue
    end
    ports = get_param(paths{k},'PortHandles');
    for n = 1:numel(ports.Outport)
        line = get_param(ports.Outport(n),'Line');
        if line ~= -1
            dest = get_param(line,'DstPortHandle');
            assert(isscalar(dest) && dest ~= -1 && is_stage_link(links,ports.Outport(n),dest), ...
                'S2 rebuild would disconnect an unrelated output on %s.',paths{k});
        end
    end
end
for path = [paths {body}]
    if getSimulinkBlockHandle(path{1}) == -1
        continue
    end
    ports = get_param(path{1},'PortHandles');
    for n = 1:numel(ports.Inport)
        line = get_param(ports.Inport(n),'Line');
        if line ~= -1
            src = get_param(line,'SrcPortHandle');
            assert(src ~= -1 && is_stage_link(links,src,ports.Inport(n)), ...
                'Input %d of %s already has a different source.',n,path{1});
        end
    end
end
for k = 1:size(links,1)
    if getSimulinkBlockHandle(links{k,3}) ~= -1
        ports = get_param(links{k,3},'PortHandles');
        n = links{k,4};
        if n <= numel(ports.Inport) && get_param(ports.Inport(n),'Line') ~= -1
            delete_line(mdl,endpoint(links{k,1},links{k,2}),endpoint(links{k,3},n));
        end
    end
end
% Place new blocks upstream relative to the existing body's actual location.
anchor = get_param(body,'Position');
for k = 1:numel(paths)
    if getSimulinkBlockHandle(paths{k}) ~= -1
        delete_block(paths{k});
    end
    rect = grid_position(1,k);
    base = grid_position(1,4);
    offset = anchor(1:2)-base(1:2);
    rect = rect+[offset offset];
    rect(4) = rect(2)+anchor(4)-anchor(2);
    add_checked('built-in/Subsystem',paths{k},rect,'Tag',owner);
end
build_mixer(paths{1});
build_reference(paths{2});
build_motors(paths{3});
set_param(paths{3},'BackgroundColor','[1.0, 0.85, 0.65]');
labels = {'Motor command (0..1)','Speed reference (rad/s)','Motor speed (rad/s)','Shaft torque (N m)'};
for k = 1:size(links,1)
    wire(mdl,endpoint(links{k,1},links{k,2}),endpoint(links{k,3},links{k,4}),labels{k});
end
save_system(mdl);
end

function found = is_stage_link(links,src,dst)
found = false;
for k = 1:size(links,1)
    if strcmp(get_param(src,'Parent'),links{k,1}) && strcmp(get_param(dst,'Parent'),links{k,3})
        a = get_param(links{k,1},'PortHandles');
        b = get_param(links{k,3},'PortHandles');
        if numel(a.Outport)>=links{k,2} && numel(b.Inport)>=links{k,4} && ...
                a.Outport(links{k,2})==src && b.Inport(links{k,4})==dst
            found = true;
            return
        end
    end
end
end

function text = endpoint(path,port)
text = sprintf('%s/%d',get_param(path,'Name'),port);
end

function build_mixer(s)
input(s,'collective',1,'1',1);
input(s,'axis_demand',2,'[3 1]',2);
block(s,'Math Operations/Gain','Collective on four motors',1,2, ...
    'Gain','ones(4,1)','Multiplication','Matrix(K*u)');
block(s,'Math Operations/Gain','Mix axes',2,2,'Gain','mixer_matrix','Multiplication','Matrix(K*u)');
block(s,'Math Operations/Sum','Combine commands',1.5,3,'Inputs','++','IconShape','rectangular');
block(s,'Discontinuities/Saturation','Limit commands',1.5,4,'LowerLimit','0','UpperLimit','1');
output(s,'motor_cmd',1,1.5,5);
% Give the sum enough height for two distinct, horizontal signal rows.
sumblock = [s '/Combine commands'];
rect = get_param(sumblock,'Position');
rect([2 4]) = rect([2 4])+[-50 50];
set_param(sumblock,'Position',rect);
align_row(s,{'collective','Collective on four motors'},sumblock,1);
align_row(s,{'axis_demand','Mix axes'},sumblock,2);
wire(s,'collective/1','Collective on four motors/1','Collective (0..1)');
wire(s,'axis_demand/1','Mix axes/1','Axis demands (fraction)');
wire(s,'Collective on four motors/1','Combine commands/1','Collective per motor (fraction)');
wire(s,'Mix axes/1','Combine commands/2','Differential commands (fraction)');
wire(s,'Combine commands/1','Limit commands/1','Requested commands (fraction)');
wire(s,'Limit commands/1','motor_cmd/1','Motor commands (0..1)');
end

function build_reference(s)
input(s,'motor_cmd',1,'[4 1]',1);
block(s,'Discontinuities/Saturation','Limit thrust command',1,2,'LowerLimit','0','UpperLimit','1');
block(s,'Math Operations/Gain','Squared speed',1,3,'Gain','max_thrust/kT_prop');
block(s,'Math Operations/Sqrt','Target speed',1,4);
% In continuous mode the Rate Limiter ignores its initial condition. Sample
% the requested speed on the solver grid so the first slew starts at hover.
block(s,'Math Operations/Reshape','Four channels',1,5,'OutputDimensionality','1-D array');
block(s,'Discrete/Zero-Order Hold','Sample target speed',1,6,'SampleTime','drive_sample_time');
block(s,'Discontinuities/Rate Limiter','Limit speed change',1,7,'SampleTimeMode','inherited', ...
    'RisingSlewLimit','speed_slew_limit','FallingSlewLimit','-speed_slew_limit','InitialCondition','hover_speed');
block(s,'Math Operations/Reshape','Speed column',1,8,'OutputDimensionality','Column vector (2-D)');
output(s,'motor_speed_ref',1,1,9);
blocks = {'motor_cmd','Limit thrust command','Squared speed','Target speed', ...
    'Four channels','Sample target speed','Limit speed change','Speed column','motor_speed_ref'};
labels = {'Thrust command (fraction)','Limited thrust command (0..1)','Squared speed (rad^2/s^2)', ...
    'Target speed (rad/s)','Speed channels (rad/s)','Sampled speed (rad/s)', ...
    'Limited speed (rad/s)','Speed reference (rad/s)'};
for k = 1:numel(labels)
    wire(s,[blocks{k} '/1'],[blocks{k+1} '/1'],labels{k});
end
end

function build_motors(s)
input(s,'motor_speed_ref',1,'[4 1]',1);
block(s,'Signal Routing/From','Read motor speed',2,1,'GotoTag','s2_speed');
block(s,'Math Operations/Sum','Speed error',1,2,'Inputs','+-','IconShape','rectangular');
block(s,'Math Operations/Gain','Motor acceleration',1,3,'Gain','1/motor_time_constant');
block(s,'Continuous/Integrator','Motor speed',1,4,'InitialCondition','hover_speed*ones(4,1)');
output(s,'motor_speed',1,1,6);
block(s,'Signal Routing/Goto','Store motor speed',1.8,4.5,'Orientation','down', ...
    'GotoTag','s2_speed','TagVisibility','local');
block(s,'Signal Routing/From','Speed for torque',3,4,'GotoTag','s2_speed');
block(s,'User-Defined Functions/MATLAB Function','Shaft torque',2.5,5);
path = [s '/Shaft torque'];
try
    rt = sfroot;
    chart = rt.find('-isa','Stateflow.EMChart','Path',path);
    chart.Script = sprintf(['function torque = fcn(dw,w)\n' ...
        'torque = kQ_prop*w.*abs(w) + J_total*dw;\nend\n']);
    if isprop(chart,'VectorOutputs1D')
        chart.VectorOutputs1D = false;
    end
    for name = {'kQ_prop','J_total'}
        d = Stateflow.Data(chart);
        d.Name = name{1};
        d.Scope = 'Parameter';
    end
    for name = {'w','dw','torque'}
        d = chart.find('-isa','Stateflow.Data','Name',name{1});
        d.Props.Array.Size = '[4 1]';
    end
catch err
    fprintf(2,'Failed to configure %s: %s\n',path,err.message);
    rethrow(err)
end
output(s,'motor_torque',2,2.5,6);
sumblock = [s '/Speed error'];
rect = get_param(sumblock,'Position');
rect([2 4]) = rect([2 4])+[-44 44];
set_param(sumblock,'Position',rect);
align_row(s,{'motor_speed_ref'},sumblock,1);
align_row(s,{'Read motor speed'},sumblock,2);
% Space the torque inputs so the acceleration branch clears the speed tag.
rect = get_param(path,'Position');
rect([2 4]) = rect([2 4])+[-50 50];
set_param(path,'Position',rect);
tagblock = [s '/Store motor speed'];
rect = get_param(tagblock,'Position');
center = round((rect(1)+rect(3))/2);
rect([1 3]) = center+[-20 20];
set_param(tagblock,'Position',rect);
align_row(s,{'Speed for torque'},path,2);
wire(s,'motor_speed_ref/1','Speed error/1','Speed reference (rad/s)');
wire(s,'Read motor speed/1','Speed error/2','Speed feedback (rad/s)');
wire(s,'Speed error/1','Motor acceleration/1','Speed error (rad/s)');
wire(s,'Motor acceleration/1','Motor speed/1','Acceleration (rad/s^2)');
wire(s,'Motor speed/1','motor_speed/1','Motor speed (rad/s)');
speed = port_position([s '/Motor speed'],'Outport',1);
tag = port_position([s '/Store motor speed'],'Inport',1);
add_line(s,[tag(1) speed(2); tag]);
accel = port_position([s '/Motor acceleration'],'Outport',1);
integrator = port_position([s '/Motor speed'],'Inport',1);
tap_x = round((accel(1)+integrator(1))/2);
torque = port_position(path,'Inport',1);
add_line(s,[tap_x accel(2); tap_x torque(2); torque]);
wire(s,'Speed for torque/1','Shaft torque/2','Motor speed for torque (rad/s)');
wire(s,'Shaft torque/1','motor_torque/1','Shaft torque (N m)');
end

function align_row(s,names,destination,port)
xy = port_position(destination,'Inport',port);
for k = 1:numel(names)
    align_port_y([s '/' names{k}],'Outport',1,xy(2));
end
end

function input(s,name,number,dim,row)
block(s,'Ports & Subsystems/In1',name,row,1,'Port',num2str(number),'PortDimensions',dim);
end

function output(s,name,number,row,col)
block(s,'Ports & Subsystems/Out1',name,row,col,'Port',num2str(number));
end

function block(s,library,name,row,col,varargin)
add_checked(['simulink/' library],[s '/' name],grid_position(row,col),varargin{:});
end

function add_checked(library,path,rect,varargin)
try
    add_block(library,path,'Position',rect,varargin{:});
catch err
    fprintf(2,'Failed to add %s: %s\n',path,err.message);
    rethrow(err)
end
end

function wire(s,from,to,name)
line = add_line(s,from,to,'autorouting','off');
set_param(line,'Name',name);
end
