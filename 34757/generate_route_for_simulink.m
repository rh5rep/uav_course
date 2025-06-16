function [route, wall_data] = generate_route_for_simulink()
    % Generate a new maze and find path
    cd maze/exercise_4_files
    generate_wall
    map_script_3d
    
    % Load the generated wall data
    load('auto_wall.txt')
    wall_data = auto_wall;
    
    % The route_scaled variable from map_script_3d will be available
    % Return both route and wall data for use in uas_parameters
    route = route_scaled;
    cd ../..
end