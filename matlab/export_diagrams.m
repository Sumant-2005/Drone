function export_diagrams()
levels = model_levels('DroneSim');
folder = fullfile('results','diagrams');
if ~isfolder(folder)
    mkdir(folder);
end
for i = 1:numel(levels)
    sys = levels{i};
    filename = sprintf('%03d_%s.png',i,matlab.lang.makeValidName(sys));
    print(['-s' sys],'-dpng',fullfile(folder,filename));
    fprintf('Exported %s\n',fullfile(folder,filename));
end
end
