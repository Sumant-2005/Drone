function controller_ports(s,n)
flight_add(s,'Reference','simulink/Ports & Subsystems/In1',grid_position(1,1),'Port','1','PortDimensions',sprintf('[%d 1]',n));
flight_add(s,'states','simulink/Ports & Subsystems/In1',grid_position(10,1),'Port','2');
% Space the reference ports vertically so each feedback row has its own lane.
rect=grid_position(1,3);
rect(3)=rect(1)+12;
rect(4)=rect(2)+n*170;
flight_add(s,'Split references','simulink/Signal Routing/Demux',rect,'Outputs',num2str(n));
center=port_position([s '/Split references'],'Inport',1);

flight_add(s,'Vector references','simulink/Math Operations/Reshape',grid_position(1,2),'OutputDimensionality','1-D array');
align_port_y([s '/Vector references'],'Outport',1,center(2));
align_port_y([s '/Reference'],'Outport',1,center(2));
flight_wire(s,'Reference/1','Vector references/1','Controller references (SI)');
flight_wire(s,'Vector references/1','Split references/1','Controller references (SI)');
flight_add(s,'Feedback bus','simulink/Signal Routing/Goto',grid_position(10,2),'GotoTag','measured','TagVisibility','local');
flight_wire(s,'states/1','Feedback bus/1','states');
end
