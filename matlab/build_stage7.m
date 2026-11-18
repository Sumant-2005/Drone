function build_stage7()
mdl='DroneSim';
assert(isfile('DroneSim.slx'),'The saved DroneSim.slx with your finished Drone body is required.');
load_system('DroneSim.slx');
assert(getSimulinkBlockHandle([mdl '/Drone body'])~=-1,'Missing finished Drone body.');
names={'Mixer','Thrust to speed','Motors (ideal, placeholder)'};
present=cellfun(@(n)getSimulinkBlockHandle([mdl '/' n])~=-1,names);
if ~any(present), build_stage2();
elseif ~all(present), error('Incomplete S2 plant. Complete its local build before assembly.'); end
controllers={'Rate control','Attitude control','Velocity and altitude control','Mission command'};
for i=1:4
    path=[mdl '/' controllers{i}];
    if getSimulinkBlockHandle(path)==-1 || ~isequal(get_param(path,'UserData'),'complete')
        feval(sprintf('build_stage%d',i+2));
    end
end
% Connect the forward path without moving established plant or body diagrams.
names={'Mission command','Velocity and altitude control','Attitude control','Rate control','Mixer', ...
    'Thrust to speed','Motors (ideal, placeholder)','Drone body'};
for i=1:7
    port=1;
    if i==4, port=2; end
    link(mdl,names{i},1,names{i+1},port);
end
link(mdl,names{7},2,names{8},2);
pos=get_param([mdl '/Mixer'],'Position');
collector=owned(mdl,'Read collective','simulink/Signal Routing/From', ...
    [pos(1)-130 pos(2)-85 pos(1)-45 pos(2)-57],'GotoTag','flight_collective');
link(mdl,get_param(collector,'Name'),1,'Mixer',1);
tap(mdl,'Velocity and altitude control',2,'collective',false);
% A single return line runs below the forward path, with one tap per controller.
p=port_position([mdl '/Drone body'],'Outport',1);
q=port_position([mdl '/Mission command'],'Inport',1);
bottom=max(p(2),q(2))+250;
dst=get_param([mdl '/Mission command'],'PortHandles');
if get_param(dst.Inport(1),'Line')==-1
    line=add_line(mdl,[p;p+[90 0];p(1)+90 bottom;q(1)-45 bottom;q(1)-45 q(2);q]);
    set_param(line,'Name','states');
end
for i=2:4
    block=[mdl '/' names{i}];
    ports=get_param(block,'PortHandles');
    if get_param(ports.Inport(2),'Line')==-1
        q=port_position(block,'Inport',2);
        add_line(mdl,[q(1)-30 bottom;q(1)-30 q(2);q]);
    end
end
for i=1:4
    port=1+(i>1); link(mdl,'Drone body',1,names{i},port);
end
tap(mdl,'Mission command',1,'command',true);
tap(mdl,'Velocity and altitude control',1,'attitude',true);
tap(mdl,'Attitude control',1,'rates',true);
tap(mdl,'Mixer',1,'cmd',true);
tap(mdl,'Thrust to speed',1,'ref',true);
tap(mdl,'Motors (ideal, placeholder)',1,'speed',true);
tap(mdl,'Motors (ideal, placeholder)',2,'torque',false);
tap(mdl,'Drone body',1,'states',true);
pos=get_param([mdl '/Mission command'],'Position');
for i=1:2
    blocks={'Waveforms','Flight path'};
    name=blocks{i};
    rect=grid_position(1,i);
    rect=rect+[pos(1) bottom+55 pos(1) bottom+55];
    path=owned(mdl,name,'built-in/Subsystem',rect);
    % These monitor diagrams have no external ports or stored flight state.
    Simulink.SubSystem.deleteContents(path);
    build_monitors(path,i);
end
if isempty(find_system(mdl,'FindAll','on','SearchDepth',1,'Type','annotation','Tag','DroneSim_S7'))
    note=Simulink.Annotation(mdl,sprintf(['Set the stop time in the toolbar, press Run, then double-click Waveforms.\n' ...
        'Flight path shows east versus north.\nThe 3-D path opens at the end of the run.']));
    note.Tag='DroneSim_S7'; note.Position=[pos(1) bottom+230];
end
save_system(mdl);
end

function link(m,source,out,dest,in)
a=get_param([m '/' source],'PortHandles');
b=get_param([m '/' dest],'PortHandles');
line=get_param(b.Inport(in),'Line');
if line==-1
    add_line(m,a.Outport(out),b.Inport(in),'autorouting','off');
else
    while get_param(line,'LineParent')~=-1, line=get_param(line,'LineParent'); end
    assert(get_param(line,'SrcPortHandle')==a.Outport(out),'Occupied input %s/%d.',dest,in);
end
end

function tap(m,source,n,tag,above)
p=port_position([m '/' source],'Outport',n);
x=p(1)+28;
if above, y=p(2)-90; orientation='up'; else, y=p(2)+90; orientation='down'; end
name=['Monitor ' tag];
g=owned(m,name,'simulink/Signal Routing/Goto',[x-20 y-14 x+20 y+14], ...
    'Orientation',orientation,'GotoTag',['flight_' tag],'TagVisibility','global');
ports=get_param(g,'PortHandles');
if get_param(ports.Inport(1),'Line')==-1
    % Tap the existing source line. The monitor block occupies a separate lane.
    add_line(m,[source '/' num2str(n)],[name '/1'],'autorouting','off');
end
end

function path=owned(m,name,library,rect,varargin)
path=[m '/' name];
if getSimulinkBlockHandle(path)==-1
    flight_add(m,name,library,rect,'Tag','DroneSim_S7',varargin{:});
else
    assert(strcmp(get_param(path,'Tag'),'DroneSim_S7'),'Unrelated block at %s.',path);
end
end
