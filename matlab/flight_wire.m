function h = flight_wire(sys,src,dst,label)
h = add_line(sys,src,dst,'autorouting','off');
if nargin == 4
    set_param(h,'Name',label);
end
end
