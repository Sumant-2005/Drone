function check_layout()
levels = model_levels('DroneSim');
ok = true;
for i = 1:numel(levels)
    sys = levels{i};
    blocks = find_system(sys,'SearchDepth',1,'FindAll','on','Type','block');
    blocks = blocks(blocks ~= get_param(sys,'Handle'));
    overlaps = 0;
    for a = 1:numel(blocks)
        p = get_param(blocks(a),'Position');
        for b = a+1:numel(blocks)
            q = get_param(blocks(b),'Position');
            overlaps = overlaps + all(max(p(1:2),q(1:2)) < min(p(3:4),q(3:4)));
        end
    end
    crossings = count_crossings(sys);
    fprintf('%s: %d blocks, %d overlaps, %d crossings\n',sys,numel(blocks),overlaps,crossings);
    if numel(blocks) > 20
        warning('Review block count in %s.',sys);
    end
    ok = ok && overlaps == 0 && crossings == 0;
end
assert(ok,'Layout has overlapping blocks or crossing lines.');
end

function total = count_crossings(sys)
lines = find_system(sys,'SearchDepth',1,'FindAll','on','Type','line');
paths = cell(size(lines));
roots = lines;
for i = 1:numel(lines)
    paths{i} = get_param(lines(i),'Points');
    parent = get_param(roots(i),'LineParent');
    while parent ~= -1
        roots(i) = parent;
        parent = get_param(parent,'LineParent');
    end
end
total = 0;
for i = 1:numel(lines)
    for j = i+1:numel(lines)
        if roots(i) == roots(j)
            continue
        end
        points = zeros(0,2);
        a = paths{i};
        b = paths{j};
        for m = 1:size(a,1)-1
            for n = 1:size(b,1)-1
                hit = intersection(a(m,:),a(m+1,:),b(n,:),b(n+1,:));
                points = [points; hit]; %#ok<AGROW>
            end
        end
        total = total + size(unique(points,'rows'),1);
    end
end
end

function point = intersection(a,b,c,d)
point = zeros(0,2);
u = b-a;
v = d-c;
if all(u==0) || all(v==0)
    return
end
den = cross2(u,v);
if den == 0
    if cross2(c-a,u) == 0
        % Collinear traces overlap if their projected intervals touch.
        [~,axis] = max(abs(u));
        ends = sort([(c(axis)-a(axis))/u(axis), (d(axis)-a(axis))/u(axis)]);
        t = max(0,ends(1));
        if t <= min(1,ends(2))
            point = a+t*u;
        end
    end
else
    t = cross2(c-a,v)/den;
    s = cross2(c-a,u)/den;
    if t>=0 && t<=1 && s>=0 && s<=1
        point = a+t*u;
    end
end
end

function z = cross2(x,y)
z = x(1)*y(2)-x(2)*y(1);
end
