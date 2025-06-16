function route = generate_simple_route(start_pos, end_pos)
    % Simple route generator for maze_1_3D using A* or Greedy pathfinding
    
    % Load the static maze
    run('maze/maze_1_3D.m');  % This loads the map variable
    
    % CRITICAL: Store original maze for validation
    original_map = map;
    
    % Debug the maze and positions
    fprintf('=== MAZE DEBUG ===\n');
    fprintf('Maze dimensions: [%d, %d, %d]\n', size(map));
    fprintf('Start position: [%d, %d, %d]\n', start_pos);
    fprintf('End position: [%d, %d, %d]\n', end_pos);
    
    % Force start and end to be free spaces
    map(start_pos(1), start_pos(2), start_pos(3)) = 0;
    map(end_pos(1), end_pos(2), end_pos(3)) = 0;
    
    % Run A* pathfinding
    fprintf('Running pathfinding...\n');
    route = astar_3d(map, start_pos, end_pos);
    
    % Check if route was found
    if isempty(route)
        error('No route found between start and end positions');
    end
    
    % DETAILED ROUTE ANALYSIS
    fprintf('=== DETAILED ROUTE ANALYSIS ===\n');
    fprintf('Checking each waypoint against original maze:\n');
    wall_collisions = 0;
    for i = 1:size(route, 1)
        pos = route(i, :);
        original_val = original_map(pos(1), pos(2), pos(3));
        layer_name = sprintf('layer%d', pos(3));
        
        % Replace ternary operator with if/else
        if original_val == 1
            status = 'WALL';
        else
            status = 'free';
        end
        
        fprintf('Waypoint %2d: [%2d,%2d,%2d] = %d (%s)\n', ...
            i, pos(1), pos(2), pos(3), original_val, status);
        
        if original_val == 1
            wall_collisions = wall_collisions + 1;
        end
    end
    
    % Check if the issue might be drone physics/movement between waypoints
    fprintf('\n=== MOVEMENT ANALYSIS ===\n');
    for i = 2:size(route, 1)
        prev_pos = route(i-1, :);
        curr_pos = route(i, :);
        movement = curr_pos - prev_pos;
        distance = norm(movement);
        fprintf('Step %2d: [%2d,%2d,%2d] -> [%2d,%2d,%2d] (move: [%2d,%2d,%2d], dist: %.1f)\n', ...
            i-1, prev_pos(1), prev_pos(2), prev_pos(3), ...
            curr_pos(1), curr_pos(2), curr_pos(3), ...
            movement(1), movement(2), movement(3), distance);
    end
    
    if wall_collisions > 0
        fprintf('\nCRITICAL: Route has %d wall collisions!\n', wall_collisions);
        error('Generated route goes through walls - check astar_3d implementation');
    else
        fprintf('\nRoute validation PASSED - no wall collisions detected.\n');
        fprintf('If drone still crashes, the issue is likely in:\n');
        fprintf('1. Drone physics/controller\n');
        fprintf('2. Coordinate system conversion in simulation\n');
        fprintf('3. Wall collision detection in simulation\n');
    end
    
    fprintf('=== ROUTE FOUND ===\n');
    fprintf('Route length: %d waypoints\n', size(route, 1));
    fprintf('Route:\n');
    disp(route);
end
