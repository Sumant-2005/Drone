function build_inner_controller(stage)
if stage==3
    s = flight_stage('Rate control',3,'Mixer');
    signals = {'p','q','r'};
else
    s = flight_stage('Attitude control',4,'Rate control');
    signals = {'roll','pitch'};
end
controller_ports(s,3);
% Vector Concatenate followed by a reshape preserves the 3-by-1 interface.
flight_add(s,'Collect demands','simulink/Signal Routing/Mux',grid_position(2,7),'Inputs','3');
labels = {'Roll','Pitch','Yaw'};
for i=1:numel(signals)
    name = labels{i};
    row = 2*i-1;
    if stage==3
        args = cellfun(@(p) sprintf('%s(%d)',p,i), ...
            {'rate_kp','rate_ki','axis_limit','rate_i_limit'},'UniformOutput',false);
        flight_row(s,name,grid_position(row,5),signals{i},'PI',args{:});
    else
        flight_row(s,name,grid_position(row,5),signals{i},'P','attitude_kp','','angle_rate_limit','');
    end
    flight_wire(s,sprintf('Split references/%d',i),[name '/1'],'Axis reference (SI)');
    controller_feedback(s,[name ' feedback'],row+0.6,4,[name '/2']);
    flight_wire(s,[name '/1'],sprintf('Collect demands/%d',i),'Axis demand (SI)');
end
if stage==4
    flight_wire(s,'Split references/3','Collect demands/3','Yaw rate demand (rad/s)');
end
flight_add(s,'Demand column','simulink/Math Operations/Reshape',grid_position(2,8),'OutputDimensionality','Column vector (2-D)');
flight_add(s,'Demand','simulink/Ports & Subsystems/Out1',grid_position(2,9));
flight_wire(s,'Collect demands/1','Demand column/1','Axis demands (SI)');
flight_wire(s,'Demand column/1','Demand/1','Axis demand column (SI)');
save_system('DroneSim');
end
