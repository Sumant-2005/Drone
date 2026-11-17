function controller_ports(s,n)
flight_add(s,'Reference','simulink/Ports & Subsystems/In1',grid_position(1,1),'Port','1','PortDimensions',sprintf('[%d 1]',n));
flight_add(s,'states','simulink/Ports & Subsystems/In1',grid_position(5,1),'Port','2');
flight_add(s,'Split references','simulink/Signal Routing/Demux',grid_position(1,3),'Outputs',num2str(n));
flight_add(s,'Vector references','simulink/Math Operations/Reshape',grid_position(1,2),'OutputDimensionality','1-D array');
flight_wire(s,'Reference/1','Vector references/1','Controller references (SI)');
flight_wire(s,'Vector references/1','Split references/1','Controller references (SI)');
flight_add(s,'Feedback bus','simulink/Signal Routing/Goto',grid_position(5,2),'GotoTag','measured','TagVisibility','local');
flight_wire(s,'states/1','Feedback bus/1','states');
end
