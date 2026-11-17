function run_stage(N)
validateattributes(N,{'numeric'},{'scalar','integer','nonnegative','finite'});
assert(isfile(fullfile('matlab','run_stage.m')), 'Run from the repository root.');
addpath('matlab');
tasks = {sprintf('build_stage%d',N),'test_params',sprintf('test_stage%d',N), ...
    'check_layout','export_diagrams'};
if N == 8
    tasks = [{'build_stage8','test_params'},arrayfun(@(j)sprintf('test_stage%d',j),1:8,'UniformOutput',false), ...
        {'check_layout','export_diagrams'}];
elseif N == 2
    tasks = [tasks(1:2), {'test_stage1'}, tasks(3:end)];
end
status = repmat({'SKIPPED'},size(tasks));
errors = {};
for i = 1:numel(tasks)
    try
        feval(tasks{i});
        status{i} = 'PASS';
    catch err
        status{i} = 'FAIL';
        errors{end+1} = err; %#ok<AGROW>
        fprintf(2,'%s\n',getReport(err,'extended','hyperlinks','off'));
        if i == 1
            break
        end
    end
end
disp(table(tasks(:),status(:),'VariableNames',{'Step','Status'}));
if ~isempty(errors)
    rethrow(errors{1});
end
end
