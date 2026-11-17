function test_stage1()
mdl = 'DroneSim';
fixture = jsondecode(fileread(fullfile('expected','stage1.json')));
% Compile the delivered model before exercising an isolated copy of its body.
set_param(mdl,'SimulationCommand','update');
report_check('DroneSim compilation','completed','completed',true);

[~,suffix] = fileparts(tempname);
harness = matlab.lang.makeValidName(['BodyTest_' suffix]);
new_system(harness,'Model');
cleanup = onCleanup(@() close_system(harness,0)); %#ok<NASGU>
set_param(harness,'SolverType',get_param(mdl,'SolverType'),'Solver',get_param(mdl,'Solver'), ...
    'FixedStep',get_param(mdl,'FixedStep'),'SaveOutput','on','OutputSaveName','yout', ...
    'SaveFormat','Dataset','ReturnWorkspaceOutputs','on');
workspace = get_param(harness,'ModelWorkspace');
evalin(workspace,'drone_parameters');
body = [harness '/Drone body'];
add_block([mdl '/Drone body'],body,'Position',grid_position(2,2));
constant = 'simulink/Sources/Constant';
add_block(constant,[harness '/Speed'],'Value','test_speed','VectorParams1D','off','Position',grid_position(1,1));
add_block(constant,[harness '/Torque'],'Value','test_torque','VectorParams1D','off','Position',grid_position(3,1));
add_block('simulink/Ports & Subsystems/Out1',[harness '/states'],'Position',grid_position(2,3));
add_line(harness,'Speed/1','Drone body/1');
add_line(harness,'Torque/1','Drone body/2');
add_line(harness,'Drone body/1','states/1');
% Read the actual derivative at t=0. Finite differences include damping.
add_block('simulink/Sinks/To Workspace',[body '/Initial derivative'], ...
    'VariableName','bodyDerivative','SaveFormat','Timeseries','Position',grid_position(1,3));
add_line(body,'Dynamics/1','Initial derivative/1');

ok = true;
names = fieldnames(fixture.cases);
for i = 1:numel(names)
    name = names{i};
    c = fixture.cases.(name);
    assignin(workspace,'initial_state',c.x0(:));
    assignin(workspace,'drag_linear',c.drag);
    assignin(workspace,'test_speed',c.speed(:));
    assignin(workspace,'test_torque',c.torque(:));
    out = sim(harness,'StopTime',num2str(c.duration,17),'ReturnWorkspaceOutputs','on');
    dataset = out.get('yout');
    bus = dataset.getElement(1).Values;
    assert(isequal(sort(fieldnames(bus)),sort(fixture.states(:))),'Unexpected states bus fields.');
    dx = getdatasamples(out.get('bodyDerivative'),1);
    got = struct();
    switch name
        case 'hover'
            got.max_state_error = zeros(size(c.x0));
            for j = 1:numel(fixture.states)
                series = bus.(fixture.states{j}).Data;
                got.max_state_error(j) = max(abs(series(:)-c.x0(j)));
            end
        case {'fall_free','fall_drag'}
            got.fall_m = bus.down.Data(end)-c.x0(3);
        case 'pitch'
            got.moment_Nm = evalin(workspace,'inertia_yy')*dx(11);
            got.qdot = dx(11);
            got.initial_down_accel = dx(6);
        case 'yaw'
            got.rdot = dx(12);
            got.roll_pitch_rate_peak = max(abs([bus.roll.Data(:);bus.pitch.Data(:); ...
                bus.p.Data(:);bus.q.Data(:)]));
        case 'ground'
            equilibrium = evalin(workspace,'drone_mass*gravity/ground_stiffness');
            down = bus.down.Data(:);
            got.down_m = down(end);
            got.overshoot_m = max(0,max(down)-equilibrium);
            got.vertical_speed_mps = bus.v_down.Data(end);
            outside = find(abs(down-equilibrium)>c.band_fraction*equilibrium,1,'last');
            got.settling_s = inf;
            if isempty(outside)
                got.settling_s = bus.down.Time(1);
            elseif outside < numel(down)
                got.settling_s = bus.down.Time(outside+1);
            end
            within = got.settling_s <= c.deadline_s;
            report_check('ground/settling deadline',num2str(c.deadline_s),num2str(got.settling_s),within);
            ok = ok && within;
    end
    metrics = fieldnames(c.checks);
    for j = 1:numel(metrics)
        key = metrics{j};
        target = c.checks.(key);
        actual = got.(key);
        passed = isequal(size(target.value),size(actual)) && all(abs(target.value(:)-actual(:))<=target.atol);
        report_check([name '/' key],mat2str(target.value,10),mat2str(actual,10),passed);
        ok = ok && passed;
    end
end
assert(ok,'S1 body comparison failed.');
end
