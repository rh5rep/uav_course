% astar_3d.m  – single-layer version
function path = astar_3d(map,start,goal)

    sz  = size(map);                   N   = prod(sz);
    ind = @(p) sub2ind(sz,p(1),p(2),p(3));

    % 4-connected offsets (constrained to one layer)
    offsets = [ 1  0  0;
               -1  0  0;
                0  1  0;
                0 -1  0];      % NOTE: no ±Z rows

    manh = @(p) norm(double(p-goal),1);      % Manhattan heuristic

    g     = inf(N,1);         g(ind(start)) = 0;
    par   = zeros(N,1,'uint32');
    open  = [ind(start)  manh(start)];       % [index  f=g+h]

    while ~isempty(open)
        [~,k] = min(open(:,2));    cur = open(k,1);   open(k,:) = [];
        if cur == ind(goal), break; end

        [r,c,l] = ind2sub(sz,cur);  p = [r c l];

        for d = 1:size(offsets,1)
            n = p + offsets(d,:);
            if any(n<1) || n(1)>sz(1) || n(2)>sz(2) || n(3)>sz(3), continue; end
            if map(n(1),n(2),n(3)), continue; end       % wall → skip
            ni = ind(n);            tentative = g(cur) + 1;
            if tentative < g(ni)
                g(ni) = tentative;   par(ni) = cur;
                open   = [open; ni  tentative + manh(n)];   %#ok<AGROW>
            end
        end
    end

    % back-track
    path = goal;   idx = ind(goal);
    while idx ~= ind(start)
        idx = par(idx);      [r,c,l] = ind2sub(sz,idx);
        path = [r c l; path];                                    %#ok<AGROW>
    end
end
