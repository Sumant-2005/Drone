function report_check(label, expected, got, passed)
status = 'FAIL';
if passed
    status = 'PASS';
end
fprintf('%s: %s | %s | %s\n', label, expected, got, status);
end
