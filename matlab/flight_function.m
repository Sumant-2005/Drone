function block = flight_function(sys,name,pos,code,parameters)
block = flight_add(sys,name,'simulink/User-Defined Functions/MATLAB Function',pos);
try
    rt = sfroot;
    chart = rt.find('-isa','Stateflow.EMChart','Path',block);
    chart.Script = code;
    for k=1:numel(parameters)
        d = Stateflow.Data(chart);
        d.Name = parameters{k};
        d.Scope = 'Parameter';
        d.Props.Array.Size = '-1';
    end
catch err
    fprintf(2,'Failed to configure %s: %s\n',block,err.message);
    rethrow(err)
end
end
