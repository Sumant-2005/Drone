function test_stage0()
mdl = 'DroneSim';
ref = jsondecode(fileread(fullfile('expected','stage0.json')));
blocks = find_system(mdl,'SearchDepth',1,'Type','block');
step = str2double(get_param(mdl,'FixedStep'));
solver = get_param(mdl,'Solver');
kind = get_param(mdl,'SolverType');
ok = [strcmp(ref.solver.value,solver), strcmp(ref.solver_type.value,kind), ...
    abs(step-ref.fixed_step_s.value)<=ref.fixed_step_s.atol, ...
    abs(numel(blocks)-ref.top_level_blocks.value)<=ref.top_level_blocks.atol];
report_check('Solver',ref.solver.value,solver,ok(1));
report_check('Solver type',ref.solver_type.value,kind,ok(2));
report_check('Step (s)',num2str(ref.fixed_step_s.value),num2str(step),ok(3));
report_check('Blocks',num2str(ref.top_level_blocks.value),num2str(numel(blocks)),ok(4));
assert(all(ok),'S0 configuration mismatch.');
sim(mdl);
report_check('Empty model simulation','completed','completed',true);
end
