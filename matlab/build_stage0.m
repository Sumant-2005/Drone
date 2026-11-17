function build_stage0()
% Run from the repository root. S0 owns the model configuration only.
mdl = 'DroneSim';
if ~bdIsLoaded(mdl)
    if isfile([mdl '.slx'])
        load_system([mdl '.slx']);
    else
        new_system(mdl, 'Model');
    end
end
try
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
        'FixedStep', '0.00125', 'PreLoadFcn', 'addpath(''matlab'')', ...
        'InitFcn', 'drone_parameters');
    save_system(mdl, [mdl '.slx']);
catch err
    fprintf(2, 'Failed to configure model %s: %s\n', mdl, err.message);
    rethrow(err)
end
end
