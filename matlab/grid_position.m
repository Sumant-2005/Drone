function rect = grid_position(row,column)
% Grid coordinates start at one. Simulink block positions are pixels.
left = 40 + (column-1)*210;
top = 40 + (row-1)*110;
rect = [left top left+140 top+48];
end
