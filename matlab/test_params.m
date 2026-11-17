function test_params()
actual = read_parameters();
expected = jsondecode(fileread(fullfile('expected','params.json')));
keys = union(fieldnames(expected), fieldnames(actual));
ok = true;
for i = 1:numel(keys)
    k = keys{i};
    if ~isfield(expected,k) || ~isfield(actual,k)
        fprintf('%s expected present=%d | got present=%d | FAIL\n', ...
            k, isfield(expected,k), isfield(actual,k));
        ok = false;
        continue
    end
    a = actual.(k);
    e = expected.(k);
    % JSON represents vectors without row/column orientation.
    if isvector(e) && isvector(a)
        e = e(:);
        a = a(:);
    end
    passed = isequal(size(e),size(a)) && all(abs(e(:)-a(:)) <= 32*eps(max(1,abs(e(:)))));
    report_check(k,mat2str(e),mat2str(a),passed);
    ok = ok && passed;
end
assert(ok, 'Parameter comparison failed.');
end

function values = read_parameters()
drone_parameters
names = who;
values = struct();
for i = 1:numel(names)
    values.(names{i}) = eval(names{i});
end
end
