function controller_feedback(s,name,row,col,target)
flight_add(s,name,'simulink/Signal Routing/From',grid_position(row,col),'GotoTag','measured');
flight_wire(s,[name '/1'],target,'states');
end
