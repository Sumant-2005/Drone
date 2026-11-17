function sys = flight_stage(name,stage,upstream)
% Place new stage relative to its installed downstream neighbor.
mdl = 'DroneSim';
assert(isfile([mdl '.slx']),'Open the saved DroneSim.slx containing the previous stages.');
load_system([mdl '.slx']);
neighbor = [mdl '/' upstream];
assert(getSimulinkBlockHandle(neighbor) ~= -1,'Missing prerequisite %s.',neighbor);
sys = [mdl '/' name];
tag = sprintf('DroneSim_S%d',stage);
if getSimulinkBlockHandle(sys) ~= -1
    assert(strcmp(get_param(sys,'Tag'),tag),'Existing %s is not owned by this stage.',sys);
    delete_block(sys);
end
rect = get_param(neighbor,'Position')-[340 0 340 0];
flight_add(mdl,name,'built-in/Subsystem',rect,'Tag',tag);
end
