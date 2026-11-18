function test_stage6()
fixture=jsondecode(fileread(fullfile('expected','stage6.json')));
drone_parameters;
[~,suffix]=fileparts(tempname);
m=matlab.lang.makeValidName(['MissionBench_' suffix]);
new_system(m,'Model');
cleanup=onCleanup(@()close_system(m,0)); %#ok<NASGU>
ws=get_param(m,'ModelWorkspace');
evalin(ws,'drone_parameters');
set_param(m,'SolverType','Fixed-step','Solver','ode4','FixedStep',get_param('DroneSim','FixedStep'));
add_block('DroneSim/Mission command',[m '/Mission'],'Position',grid_position(1,3));
flight_add(m,'Yaw','simulink/Sources/Constant',grid_position(1,1),'Value','test_yaw');
flight_add(m,'Bus','simulink/Signal Routing/Bus Creator',grid_position(1,2),'Inputs','1');
flight_add(m,'Reference','simulink/Sinks/To Workspace',grid_position(1,4),'VariableName','cmd','SaveFormat','Timeseries');
flight_wire(m,'Yaw/1','Bus/1','yaw');
flight_wire(m,'Bus/1','Mission/1','states');
flight_wire(m,'Mission/1','Reference/1');
ok=true;
for yaw=unique([fixture.samples.yaw])
    assignin(ws,'test_yaw',yaw);
    out=sim(m,'StopTime',num2str(mission_table(end,1)),'ReturnWorkspaceOutputs','on');
    ts=out.cmd;
    for j=find([fixture.samples.yaw]==yaw)
        ref=fixture.samples(j);
        [e,i]=min(abs(ts.Time-ref.t));
        got=getdatasamples(ts,i); got=got(:);
        passed=e<drive_sample_time/2 && max(abs(got-[ref.value(:);0]))<=ref.atol;
        report_check(sprintf('t=%g yaw=%g',ref.t,yaw),mat2str([ref.value(:);0],6),mat2str(got,6),passed);
        ok=ok && passed;
    end
end
assert(ok,'Mission command comparisons failed.');
end
