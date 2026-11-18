function data = flight_bench(c,installed)
% Closed-loop tests use the installed blocks. The final tests run DroneSim itself.
drone_parameters;
source = 'DroneSim';
kind = 'mission';
if isfield(c,'mode'), kind=c.mode; end
mass = drone_mass;
if isfield(c,'mass'), mass=c.mass; end
trim = sqrt(mass*gravity/(4*kT_prop));
collective = mass*gravity/(4*max_thrust);
comp = tilt_compensation;
if isfield(c,'compensation'), comp=double(c.compensation); end
table = mission_table;
if isfield(c,'table'), table=c.table; end
if ~strcmp(kind,'mission')
    table = [0 0 0 0 0; c.duration 0 0 0 0];
    if strcmp(kind,'velocity'), table(1,2:5)=reshape(c.value,1,4); table(1,5)=c.value(4)*180/pi; end
end
if installed
    mdl=source;
else
    [~,suffix]=fileparts(tempname);
    mdl=matlab.lang.makeValidName(['FlightBench_' suffix]);
    new_system(mdl,'Model');
    cleanup=onCleanup(@() close_system(mdl,0)); %#ok<NASGU>
    set_param(mdl,'SolverType','Fixed-step','Solver','ode4','FixedStep',get_param(source,'FixedStep'));
    evalin(get_param(mdl,'ModelWorkspace'),'drone_parameters');
    make_loop(mdl,source,c,kind,collective);
end
in=Simulink.SimulationInput(mdl);
in=in.setModelParameter('StopTime',num2str(c.duration,17),'SignalLogging','on','SignalLoggingName','logsout');
vars={'drone_mass',mass;'hover_speed',trim;'hover_command',collective;'tilt_compensation',comp; ...
    'mission_table',table;'show_flight_path',0};
for i=1:size(vars,1), in=in.setVariable(vars{i,1},vars{i,2},'Workspace',mdl); end

% Record at source ports even when the saved signal already branches.
taps = {'Drone body',1,'body'; 'Motors (ideal, placeholder)',1,'w'; ...
    'Motors (ideal, placeholder)',2,'torque';'Rate control',1,'axis'};
if getSimulinkBlockHandle([mdl '/Velocity and altitude control'])~=-1
    taps(end+1,:)={'Velocity and altitude control',1,'angles'};
end
for i=1:3
    rows={'Roll','Pitch','Yaw'};
    taps(end+1,:)={['Rate control/' rows{i} '/Clamped PI'],2,sprintf('integral%d',i)}; %#ok<AGROW>
end
old=cell(size(taps,1),4);
dirty=get_param(mdl,'Dirty');
for i=1:size(taps,1)
    ports=get_param([mdl '/' taps{i,1}],'PortHandles');
    p=ports.Outport(taps{i,2});
    old(i,:)={p,get_param(p,'DataLogging'),get_param(p,'DataLoggingNameMode'),get_param(p,'DataLoggingName')};
end
restore=onCleanup(@() restore_logging(mdl,dirty,old)); %#ok<NASGU>
for i=1:size(taps,1)
    set_param(old{i,1},'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',taps{i,3});
end
out=sim(in);
logs=out.logsout;
body=logs.getElement('body').Values;
% Put all signals on the same 5 ms observation grid used in Python.
t=(0:4*drive_sample_time:c.duration).';
data=struct('time',t);
names=fieldnames(body);
for i=1:numel(names), data.(names{i})=on_grid(body.(names{i}),t); end
data.speed=on_grid(logs.getElement('w').Values,t);
torque=on_grid(logs.getElement('torque').Values,t);
axis=on_grid(logs.getElement('axis').Values,t);
for i=1:3
    data.(sprintf('axis%d',i))=axis(:,i);
    data.(sprintf('integral%d',i))=on_grid(logs.getElement(sprintf('integral%d',i)).Values,t);
end
data.height=-data.down;
data.climb=-data.v_down;
data.altitude_change=data.height+initial_state(3);
data.position_error=sqrt(data.north.^2+data.east.^2+data.altitude_change.^2);
data.horizontal_speed=hypot(data.v_north,data.v_east);
data.total_kw=sum(data.speed.*torque,2)/1000;
for i=1:4
    data.(sprintf('rpm%d',i))=data.speed(:,i)*30/pi;
    data.(sprintf('torque%d',i))=torque(:,i);
    data.(sprintf('kw%d',i))=data.speed(:,i).*torque(:,i)/1000;
end
if any(strcmp(taps(:,3),'angles'))
    angles=on_grid(logs.getElement('angles').Values,t);
    data.tilt=hypot(angles(:,1),angles(:,2));
end
end

function restore_logging(mdl,dirty,old)
for i=1:size(old,1)
    set_param(old{i,1},'DataLoggingNameMode','Custom','DataLoggingName',old{i,4}, ...
        'DataLogging',old{i,2});
    set_param(old{i,1},'DataLoggingNameMode',old{i,3});
end
set_param(mdl,'Dirty',dirty);
end

function v=on_grid(ts,t)
n=numel(ts.Time);
if ts.IsTimeFirst, a=reshape(ts.Data,n,[]); else, a=reshape(ts.Data,[],n).'; end
[time,idx]=unique(ts.Time(:),'last');
if numel(time)==1, v=repmat(a(idx,:),numel(t),1); else, v=interp1(time,a(idx,:),t,'linear','extrap'); end
end

function make_loop(m,s,c,kind,collective)
names={'Drone body','Motors (ideal, placeholder)','Thrust to speed','Mixer','Rate control', ...
       'Attitude control','Velocity and altitude control','Mission command'};
count=5;
if strcmp(kind,'attitude'), count=6;
elseif ismember(kind,{'velocity','altitude'}), count=7;
elseif strcmp(kind,'mission'), count=8;
end
for i=1:count
    add_block([s '/' names{i}],[m '/' names{i}],'Position',grid_position(1,10-i));
end
flight_wire(m,'Motors (ideal, placeholder)/1','Drone body/1');
flight_wire(m,'Motors (ideal, placeholder)/2','Drone body/2');
flight_wire(m,'Thrust to speed/1','Motors (ideal, placeholder)/1');
flight_wire(m,'Mixer/1','Thrust to speed/1');
flight_wire(m,'Rate control/1','Mixer/2');
for i=5:min(count,7), flight_wire(m,'Drone body/1',[names{i} '/2']); end
for i=6:count, flight_wire(m,[names{i} '/1'],[names{i-1} '/1']); end
if count<7
    flight_add(m,'Bench angles','simulink/Signal Routing/Bus Selector',grid_position(2,3),'OutputSignals','roll,pitch');
    flight_wire(m,'Drone body/1','Bench angles/1');
    code=sprintf('function c = trim(phi,theta)\nc=%.17g/max(0.1,cos(phi)*cos(theta));\nend',collective);
    flight_function(m,'Trim',grid_position(2,4),code,{});
    flight_wire(m,'Bench angles/1','Trim/1');
    flight_wire(m,'Bench angles/2','Trim/2');
    flight_wire(m,'Trim/1','Mixer/1');
else
    flight_wire(m,'Velocity and altitude control/2','Mixer/1');
end
if count==8
    flight_wire(m,'Drone body/1','Mission command/1');
    return
end
v=c.value(:);
if strcmp(kind,'altitude'), v=[zeros(4,1); v]; end
if strcmp(kind,'velocity'), v=[v;0]; end
if isfield(c,'pulse_end')
    flight_add(m,'Reference','simulink/Sources/Step',grid_position(1,1),'Time',num2str(c.pulse_end,17), ...
        'Before',mat2str(v,17),'After',mat2str(zeros(size(v))),'VectorParams1D','off','SampleTime','drive_sample_time');
else
    flight_add(m,'Reference','simulink/Sources/Constant',grid_position(1,1),'Value',mat2str(v,17),'VectorParams1D','off');
end
flight_wire(m,'Reference/1',[names{count} '/1']);
end
