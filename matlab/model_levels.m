function levels = model_levels(mdl)
% Report the top-level diagram and user-owned subsystem diagrams.
subsystems = find_system(mdl,'LookUnderMasks','none','FollowLinks','off', ...
    'BlockType','SubSystem');
% MATLAB Function charts have a SubSystem block type, but their contents
% are Stateflow code, not user-drawn block diagrams for print/check_layout.
rt = sfroot;
charts = rt.find('-isa','Stateflow.EMChart');
for i = 1:numel(charts)
    subsystems(strcmp(subsystems,charts(i).Path)) = [];
end
levels = [{mdl}; subsystems(:)];
end
