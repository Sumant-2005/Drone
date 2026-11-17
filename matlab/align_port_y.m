function align_port_y(block,side,index,y)
xy = port_position(block,side,index);
dy = y-xy(2);
set_param(block,'Position',get_param(block,'Position')+[0 dy 0 dy]);
end
