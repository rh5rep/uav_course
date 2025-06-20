% astar_3d.m  – single-layer version
function path = astar_3d(map,start,goal)

    sz  = size(map);                   N   = prod(sz);
    ind = @(p) sub2ind(sz,p(1),p(2),p(3));

    % 6-connected offsets (allowing movement in Z-direction)
    offsets = [ 1  0  0;    % north
               -1  0  0;    % south
                0  1  0;    % east
                0 -1  0;    % west
                0  0  1;    % up
                0  0 -1];   % down

    manh = @(p) norm(double(p-goal),1);      % Manhattan heuristic

    g     = inf(N,1);         g(ind(start)) = 0;
    par   = zeros(N,1,'uint32');
    open  = [ind(start)  manh(start)];       % [index  f=g+h]

    function valid = is_position_valid(pos)
        r = pos(1); c = pos(2); l = pos(3);
        % Check bounds
        if r < 1 || r > sz(1) || c < 1 || c > sz(2) || l < 1 || l > sz(3)
            valid = false;
            return;
        end
        % Check if current position is a wall
        if map(r, c, l) == 1
            valid = false;
            return;
        end
        % Check for walls in the column below
        for check_l = 1:(l-1)
            if map(r, c, check_l) == 1
                valid = false;
                return;
            end
        end
        valid = true;
    end

    while ~isempty(open)
        [~,k] = min(open(:,2));    cur = open(k,1);   open(k,:) = [];
        if cur == ind(goal), break; end

        [r,c,l] = ind2sub(sz,cur);  p = [r c l];

        for d = 1:size(offsets,1)
            n = p + offsets(d,:);
            if ~is_position_valid(n)
                continue;
            end
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
