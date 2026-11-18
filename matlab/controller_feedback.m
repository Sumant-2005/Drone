function controller_feedback(s,name,row,col,target)
flight_add(s,name,'simulink/Signal Routing/From',grid_position(row,col),'GotoTag','measured');
% Place the feedback source below the reference so the two wires stay ordered.
parts=strsplit(target,'/');
xy=port_position([s '/' parts{1}],'Inport',str2double(parts{2}));
align_port_y([s '/' name],'Outport',1,xy(2)+45);
flight_wire(s,[name '/1'],target,'states');
end
