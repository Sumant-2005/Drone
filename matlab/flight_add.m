function path = flight_add(sys,name,library,position,varargin)
path = [sys '/' name];
try
    add_block(library,path,'Position',position,varargin{:});
catch err
    fprintf(2,'Failed to add %s: %s\n',path,err.message);
    rethrow(err)
end
end
