function sys = flight_stage(name,stage,upstream)
% Place new stage relative to its installed downstream neighbor.
mdl = 'DroneSim';
assert(isfile([mdl '.slx']),'Open the saved DroneSim.slx containing the previous stages.');
load_system([mdl '.slx']);
neighbor = [mdl '/' upstream];
assert(getSimulinkBlockHandle(neighbor) ~= -1,'Missing prerequisite %s.',neighbor);
sys = [mdl '/' name];
tag = sprintf('DroneSim_S%d',stage);
rect = get_param(neighbor,'Position')-[340 0 340 0];
links = cell(0,2);
if getSimulinkBlockHandle(sys) ~= -1
    assert(strcmp(get_param(sys,'Tag'),tag),'Existing %s is not owned by this stage.',sys);
    rect = get_param(sys,'Position');
    pending = get_param(sys,'UserData');
    if iscell(pending), links=pending; end
    ports = get_param(sys,'PortHandles');
    for dst=ports.Inport(:).'
        line=get_param(dst,'Line');
        if line==-1, continue; end
        while get_param(line,'LineParent')~=-1, line=get_param(line,'LineParent'); end
        src=get_param(line,'SrcPortHandle');
        if src~=-1, links(end+1,:)={endpoint(src),endpoint(dst)}; end %#ok<AGROW>
    end
    for src=ports.Outport(:).'
        line=get_param(src,'Line');
        if line==-1, continue; end
        for dst=reshape(get_param(line,'DstPortHandle'),1,[])
            if dst~=-1, links(end+1,:)={endpoint(src),endpoint(dst)}; end %#ok<AGROW>
        end
    end
    for k=1:size(links,1)
        if iscell(pending) && k<=size(pending,1), continue; end
        delete_line(mdl,links{k,1},links{k,2});
    end
    delete_block(sys);
end
flight_add(mdl,name,'built-in/Subsystem',rect,'Tag',tag,'UserData',links);
end

function text=endpoint(handle)
block=get_param(handle,'Parent');
text=sprintf('%s/%d',get_param(block,'Name'),get_param(handle,'PortNumber'));
end
