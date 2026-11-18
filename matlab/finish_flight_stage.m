function finish_flight_stage(sys)
% Restore external connections when explicitly rebuilding an installed stage.
links = get_param(sys,'UserData');
mdl = bdroot(sys);
for k = 1:size(links,1)
    add_line(mdl,links{k,1},links{k,2},'autorouting','off');
end
set_param(sys,'UserData','complete','UserDataPersistent','on');
save_system(mdl);
end
