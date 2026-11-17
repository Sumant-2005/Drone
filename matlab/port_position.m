function xy = port_position(block,side,index)
ports = get_param(block,'PortHandles');
handles = ports.(side);
xy = get_param(handles(index),'Position');
end
