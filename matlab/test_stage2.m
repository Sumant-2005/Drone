function test_stage2()
spec = jsondecode(fileread(fullfile('expected','stage2.json')));
mdl = 'DroneSim';
blocks = {'Mixer','Thrust to speed','Motors (ideal, placeholder)','Drone body'};
% Check the saved diagram's actual endpoints as well as the bench response.
verify_link(mdl,blocks{1},1,blocks{2},1);
verify_link(mdl,blocks{2},1,blocks{3},1);
verify_link(mdl,blocks{3},1,blocks{4},1);
verify_link(mdl,blocks{3},2,blocks{4},2);
ok = true;
names = fieldnames(spec.cases);
for i = 1:numel(names)
    name = names{i};
    c = spec.cases.(name);
    got = run_case(mdl,blocks,c,spec.step_time);
    metrics = fieldnames(c.checks);
    for j = 1:numel(metrics)
        metric = metrics{j};
        target = c.checks.(metric);
        actual = got.(metric);
        passed = isequal(size(actual),size(target.value)) && ...
            all(abs(actual(:)-target.value(:))<=target.atol);
        report_check([name '/' metric],mat2str(target.value,10),mat2str(actual,10),passed);
        ok = ok && passed;
    end
end
assert(ok,'S2 comparison failed.');
end

function verify_link(mdl,src,out,dst,in)
a = get_param([mdl '/' src],'PortHandles');
b = get_param([mdl '/' dst],'PortHandles');
line = get_param(b.Inport(in),'Line');
while line ~= -1 && get_param(line,'LineParent') ~= -1
    line = get_param(line,'LineParent');
end
passed = line ~= -1 && get_param(line,'SrcPortHandle') == a.Outport(out);
report_check(sprintf('%s/%d to %s/%d',src,out,dst,in),'connected',mat2str(passed),passed);
assert(passed,'S2 saved-model connection missing or incorrect.');
end

function got = run_case(mdl,blocks,c,step_time)
[~,suffix] = fileparts(tempname);
h = matlab.lang.makeValidName(['DriveTest_' suffix]);
new_system(h,'Model');
cleanup = onCleanup(@() close_system(h,0)); %#ok<NASGU>
set_param(h,'SolverType',get_param(mdl,'SolverType'),'Solver',get_param(mdl,'Solver'), ...
    'FixedStep',get_param(mdl,'FixedStep'),'ReturnWorkspaceOutputs','on');
ws = get_param(h,'ModelWorkspace');
evalin(ws,'drone_parameters');
assignin(ws,'step_time',step_time);
switch c.kind
    case 'mixer'
        indices = 1;
    case 'thrust'
        indices = 2;
    case 'motor'
        indices = 3;
    case 'chain'
        indices = 1:4;
end
for i = indices
    add_block([mdl '/' blocks{i}],[h '/' blocks{i}],'Position',grid_position(2,i+1));
end
if ismember(c.kind,{'mixer','chain'})
    assignin(ws,'test_collective',c.collective);
    assignin(ws,'test_axis',c.axis(:));
    constant(h,'Collective','test_collective',1);
    if strcmp(c.kind,'chain')
        step(h,'Demand','zeros(3,1)','test_axis',3);
    else
        constant(h,'Demand','test_axis',3);
    end
    add_line(h,'Collective/1',[blocks{1} '/1']);
    add_line(h,'Demand/1',[blocks{1} '/2']);
end
switch c.kind
    case 'mixer'
        log_output(h,[blocks{1} '/1'],'command',1,4);
    case 'thrust'
        assignin(ws,'test_command',c.command(:));
        step(h,'Command','hover_command*ones(4,1)','test_command',2);
        add_line(h,'Command/1',[blocks{2} '/1']);
        log_output(h,[blocks{2} '/1'],'reference',2,4);
    case 'motor'
        assignin(ws,'test_delta',c.delta);
        step(h,'Reference','hover_speed*ones(4,1)','(hover_speed+test_delta)*ones(4,1)',2);
        add_line(h,'Reference/1',[blocks{3} '/1']);
        log_output(h,[blocks{3} '/1'],'speed',1,5);
        log_output(h,[blocks{3} '/2'],'torque',3,5);
    case 'chain'
        for i = 1:3
            add_line(h,[blocks{i} '/1'],[blocks{i+1} '/1']);
        end
        add_line(h,[blocks{3} '/2'],[blocks{4} '/2']);
        log_output([h '/' blocks{4}],'Dynamics/1','derivative',1,3);
end
out = sim(h,'StopTime',num2str(c.duration,17),'ReturnWorkspaceOutputs','on');
switch c.kind
    case 'mixer'
        [~,cmd] = samples(out,'command');
        got.command = cmd(end,:).';
        got.command_sum = sum(got.command);
    case 'thrust'
        [t,ref] = samples(out,'reference');
        got.final_ref = ref(end,:).';
        got.peak_slew = max(abs(diff(ref)./diff(t)),[],1).';
    case 'motor'
        [t,w] = samples(out,'speed');
        [tt,torque] = samples(out,'torque');
        hover = evalin(ws,'hover_speed');
        if c.delta == 0
            got.speed_error = max(abs(w(:)-hover));
            got.torque = torque(end,:).';
            got.torque_drift = max(abs(torque-torque(1,:)),[],'all');
        else
            tau = evalin(ws,'motor_time_constant');
            fraction = (w(:,1)-hover)/c.delta;
            goal = 1-exp(-1);
            k = find(fraction>=goal,1);
            got.tau_s = NaN;
            if ~isempty(k) && k>1
                got.tau_s = interp1(fraction(k-1:k),t(k-1:k),goal)-step_time;
            end
            got.fraction_at_tau = interp1(t,fraction,step_time+tau);
            [~,k] = min(abs(tt-step_time));
            got.torque_at_step = torque(k,:).';
        end
    case 'chain'
        [~,dx] = samples(out,'derivative');
        got.axis_accel = dx(end,9+c.axis_number);
        got.positive_accel = double(got.axis_accel>0);
end
end

function constant(h,name,value,row)
add_block('simulink/Sources/Constant',[h '/' name],'Value',value,'VectorParams1D','off', ...
    'Position',grid_position(row,1));
end

function step(h,name,before,after,row)
add_block('simulink/Sources/Step',[h '/' name],'Time','step_time','Before',before,'After',after, ...
    'SampleTime','drive_sample_time','VectorParams1D','off','Position',grid_position(row,1));
end

function log_output(h,from,name,row,col)
add_block('simulink/Sinks/To Workspace',[h '/' name],'VariableName',name,'SaveFormat','Timeseries', ...
    'Position',grid_position(row,col));
add_line(h,from,[name '/1']);
end

function [t,data] = samples(out,name)
ts = out.get(name);
t = ts.Time(:);
if ts.IsTimeFirst
    data = reshape(ts.Data,numel(t),[]);
else
    data = reshape(ts.Data,[],numel(t)).';
end
end
