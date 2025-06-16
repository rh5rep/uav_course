function path = greedy_3d(map, start_pos, goal_pos)
% Path = greedy_3d(logicalMap, start, goal)
% start, goal in 0-based [x y z]

% ---- setup --------------------------------------------------------------
[Ny,Nx,Nz] = size(map);                 % note: MATLAB row = Y

start = round(start_pos) + 1;           % → 1-based
goal  = round(goal_pos)  + 1;

heuristic = @(p) norm(p - goal);

open   = [start, heuristic(start), -1]; % [y x z h parentRow]
closed = [];

% ---- main loop ----------------------------------------------------------
while ~isempty(open)
    [~,idx] = min(open(:,4));           % lowest h
    curr    = open(idx,:); open(idx,:) = [];
    if isequal(curr(1:3), goal)         % reached!
        path = backtrack(closed, curr) - 1;  % → 0-based
        return
    end
    closed  = [closed; curr];
    parent  = size(closed,1);

    N6 = [ 1 0 0;-1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
    for n = 1:6
        nb = curr(1:3) + N6(n,:);
        if any(nb < 1) || nb(1)>Ny || nb(2)>Nx || nb(3)>Nz
            continue;                               % OOB
        end
        if map(nb(1),nb(2),nb(3)), continue; end    % wall
        if ismember(nb, open(:,1:3),'rows'), continue; end
        open = [open; nb, heuristic(nb), parent];
    end
end
error('No path found');
end

% helpers ---------------------------------------------------------------
function p = backtrack(closed, node)
    p = node(1:3);
    idx = node(5);
    while idx > 0
        node = closed(idx,:);
        p    = [node(1:3); p];
        idx  = node(5);
    end
end
