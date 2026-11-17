function build_stage1()
mdl = 'DroneSim';
if ~isfile([mdl '.slx']) && ~bdIsLoaded(mdl)
    build_stage0();
else
    load_system(mdl);
end
sys = [mdl '/Drone body'];
if getSimulinkBlockHandle(sys) ~= -1
    delete_block(sys);
end
put('built-in/Subsystem',sys,grid_position(1,1));
src = @(name) [sys '/' name];
io = 'simulink/Ports & Subsystems/';
put([io 'In1'],src('motor_speed'),grid_position(1,1),'Port','1','PortDimensions','[4 1]','Unit','rad/s');
put([io 'In1'],src('motor_torque'),grid_position(2,1),'Port','2','PortDimensions','[4 1]','Unit','N*m');
put('simulink/Signal Routing/From',src('Read body state'),grid_position(3,1),'GotoTag','s1_body_state');

blk = src('Dynamics');
put('simulink/User-Defined Functions/MATLAB Function',blk,grid_position(2,2));
try
    rt = sfroot;
    chart = rt.find('-isa','Stateflow.EMChart','Path',blk);
    if isprop(chart,'VectorOutputs1D')
        chart.VectorOutputs1D = false;  % Preserve two-dimensional columns.
    end
    chart.Script = fileread(fullfile('matlab','body_dynamics.m'));
    parameters = {'kT_prop','motor_x','motor_y','motor_spin_dir','drone_mass','gravity', ...
        'drag_linear','ground_stiffness','ground_damping','ground_side_friction', ...
        'ground_level_stiffness','ground_level_damping','inertia_xx','inertia_yy', ...
        'inertia_zz','drag_rotational'};
    for i = 1:numel(parameters)
        data = Stateflow.Data(chart);
        data.Name = parameters{i};
        data.Scope = 'Parameter';
        if ismember(parameters{i},{'motor_x','motor_y','motor_spin_dir'})
            data.Props.Array.Size = '[1 4]';
        end
    end
    signals = {'motor_speed','motor_torque','x','xdot'};
    sizes = {'[4 1]','[4 1]','[12 1]','[12 1]'};
    for i = 1:numel(signals)
        data = chart.find('-isa','Stateflow.Data','Name',signals{i});
        data.Props.Array.Size = sizes{i};
    end
catch err
    fprintf(2,'Failed to configure %s: %s\n',blk,err.message);
    rethrow(err)
end
put('simulink/Continuous/Integrator',src('Integrate body state'),grid_position(2,3), ...
    'InitialCondition','initial_state');
put('simulink/Signal Routing/Goto',src('Store body state'),grid_position(4,4), ...
    'GotoTag','s1_body_state','TagVisibility','local');

% Tall, aligned routing blocks leave room for twelve horizontal bus labels.
% Keep the integrated column for feedback. Demux consumes a 1-D vector.
put('simulink/Math Operations/Reshape',src('Flatten state vector'),grid_position(2,4), ...
    'OutputDimensionality','1-D array');
split = grid_position(2,5);
bus = grid_position(2,6);
split([2 4]) = split([2 4]) + [-105 105];
bus([2 4]) = split([2 4]);
split(3) = split(1)+12;
bus(3) = bus(1)+12;
put('simulink/Signal Routing/Demux',src('Separate states'),split,'Outputs','12');
put('simulink/Signal Routing/Bus Creator',src('Build states bus'),bus,'Inputs','12');
put([io 'Out1'],src('states'),grid_position(2,7),'Port','1');

wire(sys,'motor_speed/1','Dynamics/1','motor speed (rad/s)');
wire(sys,'motor_torque/1','Dynamics/2','motor torque (N m)');
wire(sys,'Read body state/1','Dynamics/3','body state (m, m/s, rad, rad/s)');
wire(sys,'Dynamics/1','Integrate body state/1','state derivative (SI/s)');
wire(sys,'Integrate body state/1','Flatten state vector/1','integrated state (SI)');
wire(sys,'Flatten state vector/1','Separate states/1','state vector (SI)');
add_line(sys,'Integrate body state/1','Store body state/1','autorouting','off');
names = {'north','east','down','v_north','v_east','v_down','roll','pitch','yaw','p','q','r'};
units = {'m','m','m','m/s','m/s','m/s','rad','rad','rad','rad/s','rad/s','rad/s'};
for i = 1:numel(names)
    % The exact wire name becomes the bus field. Show its unit beside the fan-out.
    wire(sys,sprintf('Separate states/%d',i),sprintf('Build states bus/%d',i),names{i});
end
set_param(src('Separate states'),'AttributesFormatString', ...
    sprintf('Position (%s), velocity (%s)\nAngles (%s), body rates (%s)', ...
    units{1},units{4},units{7},units{10}));
wire(sys,'Build states bus/1','states/1','states');
save_system(mdl);
end

function put(library,path,position,varargin)
try
    add_block(library,path,'Position',position,varargin{:});
catch err
    fprintf(2,'Failed to add %s: %s\n',path,err.message);
    rethrow(err)
end
end

function wire(sys,from,to,name)
h = add_line(sys,from,to,'autorouting','off');
set_param(h,'Name',name);
end
