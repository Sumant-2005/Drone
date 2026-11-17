function build_monitors(s,kind)
if kind==2
    path_monitor(s);
    return
end
specs = { ...
    'Velocity and command', {'command','states'}, {'v_north','v_east','v_down'}, 'y=[a(1:3);b;c;-d];'; ...
    'Altitude in metres', {'states'}, {'down'}, 'y=-a;'; ...
    'Attitude in degrees', {'attitude','states'}, {'roll','pitch'}, 'y=[a(1:2);b;c]*180/pi;'; ...
    'Body rates in degrees per second', {'rates','states'}, {'p','q','r'}, 'y=[a(:);b;c;d]*180/pi;'; ...
    'Mixer commands', {'cmd'}, {}, 'y=a(:);'; ...
    'Motor and reference rpm', {'speed','ref'}, {}, 'y=[a(:);b(:)]*30/pi;'; ...
    'Differential speed reference rpm', {'ref'}, {}, 'y=(a(:)-sum(a)/4)*30/pi;'; ...
    'Shaft torque in N m', {'torque'}, {}, 'y=a(:);'; ...
    'Shaft power in kW', {'speed','torque'}, {}, 'y=a(:).*b(:)/1000;'; ...
    'Mission command', {'command'}, {}, 'y=[a(1:3);a(4)*180/pi];'};
for i=1:size(specs,1)
    row=flight_add(s,specs{i,1},'built-in/Subsystem',grid_position(i,1));
    tags=specs{i,2}; fields=specs{i,3}; sources={};
    for k=1:numel(tags)
        name=['Read ' tags{k}];
        flight_add(row,name,'simulink/Signal Routing/From',grid_position(k,1),'GotoTag',['flight_' tags{k}]);
        if strcmp(tags{k},'states')
            flight_add(row,'Measurements','simulink/Signal Routing/Bus Selector',grid_position(k,2),'OutputSignals',strjoin(fields,','));
            flight_wire(row,[name '/1'],'Measurements/1','states');
            for j=1:numel(fields), sources{end+1}=sprintf('Measurements/%d',j); end %#ok<AGROW>
        else
            sources{end+1}=[name '/1']; %#ok<AGROW>
        end
    end
    args={'a','b','c','d'};
    code=sprintf('function y = units(%s)\n%s\nend',strjoin(args(1:numel(sources)),','),specs{i,4});
    flight_function(row,'Display units',grid_position(1,3),code,{});
    for j=1:numel(sources), flight_wire(row,sources{j},sprintf('Display units/%d',j)); end
    flight_add(row,'Scope','simulink/Sinks/Scope',grid_position(1,4),'OpenAtSimulationStart','off');
    flight_wire(row,'Display units/1','Scope/1',specs{i,1});
end
end

function path_monitor(s)
flight_add(s,'Read position','simulink/Signal Routing/From',grid_position(1,1),'GotoTag','flight_states');
flight_add(s,'Position','simulink/Signal Routing/Bus Selector',grid_position(1,2),'OutputSignals','east,north,down');
flight_wire(s,'Read position/1','Position/1','states');
xy=flight_add(s,'Ground track','simulink/Sinks/XY Graph',grid_position(1,5));
if isfield(get_param(xy,'DialogParameters'),'OpenAtSimulationStart'), set_param(xy,'OpenAtSimulationStart','off'); end
flight_wire(s,'Position/1','Ground track/1','East (m)');
flight_wire(s,'Position/2','Ground track/2','North (m)');
code=sprintf('function xyz = coordinates(e,n,d)\nxyz=[e;n;-d];\nend');
flight_function(s,'East north height',grid_position(3,4),code,{});
for j=1:3, flight_wire(s,sprintf('Position/%d',j),sprintf('East north height/%d',j)); end
flight_add(s,'Draw path at stop','simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    grid_position(3,5),'FunctionName','draw_flight_path','Parameters','show_flight_path,drive_sample_time');
flight_wire(s,'East north height/1','Draw path at stop/1','East north height (m)');
end
