function check_flight_cases(stage)
fixture=jsondecode(fileread(fullfile('expected',sprintf('stage%d.json',stage))));
ok=true;
traces=struct();
for i=1:numel(fixture.cases)
    c=fixture.cases(i);
    data=flight_bench(c.inputs,stage>=7);
    traces.(c.name)=data.speed;
    for j=1:numel(c.checks)
        m=c.checks(j);
        mask=data.time>=m.start-1e-10 & data.time<=m.stop+1e-10;
        t=data.time(mask);
        y=data.(m.signal); y=y(mask);
        switch m.op
            case 'end', got=y(end);
            case 'max', got=max(y);
            case 'min', got=min(y);
            case 'peak', got=max(abs(y));
            case 'change', got=y(end)-y(1);
            case 'overshoot', got=max(0,100*(max(y/m.target)-1));
            case 'rise', got=crossing(t,y/m.target,.9)-crossing(t,y/m.target,.1);
            case 'tau', got=crossing(t,y/m.target,1-exp(-1));
            otherwise, error('Unknown metric %s',m.op);
        end
        within=abs(got-m.value)<=m.atol+m.rtol*abs(m.value);
        passed=within && got>=m.low-1e-12 && got<=m.high+1e-12;
        report_check([c.name '/' m.signal '/' m.op],num2str(m.value,10),num2str(got,10),passed);
        ok=ok && passed;
    end
end
if isfield(fixture,'mirrors')
    drone_parameters;
    for i=1:numel(fixture.mirrors)
        m=fixture.mirrors(i);
        a=traces.(m.first)-hover_speed; b=traces.(m.second)-hover_speed;
        if m.axis==2, residual=a+b;
        elseif m.axis==3, residual=a-b(:,[3 4 1 2]);
        else, residual=a-b(:,[2 1 4 3]); end
        got=max(abs(residual),[],'all')/max(abs(a),[],'all');
        passed=got<=m.limit && abs(got-m.relative_error)<=m.atol;
        report_check([m.first ' motor mirror'],num2str(m.relative_error),num2str(got),passed);
        ok=ok && passed;
    end
end
assert(ok,'S%d flight comparisons failed.',stage);
end

function v=crossing(t,y,level)
i=find(y>=level,1);
if isempty(i), v=NaN;
elseif i==1, v=t(1);
else, v=interp1(y(i-1:i),t(i-1:i),level);
end
end
