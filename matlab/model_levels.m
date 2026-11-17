function levels = model_levels(mdl)
% Report the top-level diagram and user-owned subsystem diagrams.
subsystems = find_system(mdl,'LookUnderMasks','none','FollowLinks','off', ...
    'BlockType','SubSystem');
levels = [{mdl}; subsystems(:)];
end
