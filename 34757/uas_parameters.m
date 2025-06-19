%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% UAS – Maze mission main script
% (c) 2024  — MIT Licence
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIALIZATION
clear; close all; clc

% Folder that holds your maze-definition *.m files
MAZE_DIR = '/Users/rami/Documents/DTU/Semester 2/UAS/uav_course/34757/maze';
addpath(MAZE_DIR);

% Load one of the predefined 3-D mazes — this sets variable `map`
maze_1_3D          % ↩️  edit if you want a different maze

%% 0️⃣  Utility lambdas  (insert right after maze_1_3D is called)
Ny = size(map,1);                    % number of rows in the maze

% map-index  ⟷  Gazebo-world transforms (cell centres)
idx2world = @(idx) [ (idx(:,2)-0.5) ,  (Ny-idx(:,1)+0.5) ,  idx(:,3) ];
world2idx = @(pos) [  Ny-pos(:,2)   ,  pos(:,1)+1        ,  pos(:,3) ];

% -------------------------------------------------------------------------
% 1️⃣  Define start / goal in 0-based **world** coordinates (metres, cells…)
start_pos_w = [0 0 1];
goal_pos_w  = [3 9 1];

% 2️⃣  Convert to map indices  (REPLACE the +[1 1 0] line)
start_idx = world2idx(start_pos_w);
goal_idx  = world2idx(goal_pos_w);

% -------------------------------------------------------------------------
% 3️⃣  Choose path-planner and generate the route (index space)
use_astar = true;          % set false to fall back on greedy_3d

if use_astar
    route_idx = astar_3d(map, start_idx, goal_idx);
else
    route_idx = greedy_3d(map, start_idx, goal_idx);
end

% Convert back to Gazebo world coordinates
route_world = idx2world(route_idx);  % Center of each cell

% Generate smooth trajectory using custom spline-based generator
total_time = 10.0; % Total time for the trajectory (seconds)
dt = 0.3; % Time step for sampling

% --- CHOOSE TRAJECTORY GENERATION METHOD ---
USE_BSPLINE_TRAJECTORY = true; % Set to true for B-splines, false for original interpolating spline

% Define map dimensions needed for collision checking
Nx = size(map, 2); % Number of columns in the maze
Nz = size(map, 3); % Number of layers in the maze

% Helper function for collision checking
check_collisions = @(trajectory_points) check_trajectory_collisions(trajectory_points, map, world2idx, Ny, Nx, Nz);

% First attempt with original waypoints
route_world_current = route_world;
collision_free = false;
attempt = 1;
max_attempts = 10;

while ~collision_free && attempt <= max_attempts
    if USE_BSPLINE_TRAJECTORY
        disp(['Generating trajectory with B-splines (attempt ', num2str(attempt), ')...']);
        poly_traj = bspline_trajectory_gen(route_world_current, total_time, dt);
        disp('B-spline trajectory generated.');
    else
        disp(['Generating trajectory with original interpolating spline (attempt ', num2str(attempt), ')...']);
        poly_traj = simple_trajectory_gen(route_world_current, total_time, dt);
    end
    
    % Sample the trajectory at regular intervals
    route_smooth = poly_traj(:, 2:4);  % Extract position data
    
    % Check for collisions
    [collided_indices, has_collision] = check_collisions(route_smooth);
    
    if ~has_collision
        collision_free = true;
        disp(['Collision-free trajectory found on attempt ', num2str(attempt), '.']);
    else
        warning(['Attempt ', num2str(attempt), ' failed. Trajectory collided at map indices:']);
        disp(collided_indices);
        
        if attempt < max_attempts
            if attempt <= 3
                % First few attempts: add density
                disp('Generating denser waypoints to avoid obstacles...');
                route_world_current = generate_denser_waypoints(route_world_current);
            else
                % Later attempts: use obstacle-avoiding waypoints
                disp('Generating obstacle-avoiding waypoints...');
                route_world_current = generate_obstacle_avoiding_waypoints(route_world_current, map, world2idx, idx2world, Ny, Nx, Nz);
            end
        else
            error('Failed to generate collision-free trajectory after maximum attempts.');
        end
    end
    
    attempt = attempt + 1;
end

route = route_smooth;  % Use smoothed trajectory

disp('Route (array indices:  [row col layer])');
disp(route_idx);

disp('Route (world coords, same units as start_pos_w)');
disp(route);

% Check if any waypoint sits on a wall cell
onWall = arrayfun(@(k) ...
          map(route_idx(k,1), route_idx(k,2), route_idx(k,3)), ...
          1:size(route_idx,1));
if any(onWall)
    warning('‼️  Some waypoints are inside walls at these indices:');
    disp(find(onWall));
end

% Quick 2-D visualisation of the first layer
figure(99); clf
imshow(~map(:,:,start_idx(3)));   % white = free, black = wall
hold on; grid on
plot(route_idx(:,2), route_idx(:,1), 'r-o', 'LineWidth', 2, ...
     'MarkerFaceColor', 'y');
title('Planned route on Layer 1 (row/col view)');
x0 = route_world(1,1);
y0 = route_world(1,2);
z0 = route_world(1,3);
% Timing constants required by Simulink
sample_time  = 4e-2;       % 40 ms  ≙ 25 Hz loop rate
publish_rate = sample_time;

% --- NEW: B-spline trajectory generation function ---
function traj = bspline_trajectory_gen(control_points, total_time, dt, b_spline_order)
    if nargin < 4
        b_spline_order = 4; % Default to cubic B-splines
    end

    n_control_points = size(control_points, 1);
    dims = size(control_points, 2);

    if n_control_points == 0
        traj = zeros(0, dims + 1); % No control points, empty trajectory
        return;
    end

    if n_control_points < b_spline_order
        warning('Number of control points is less than B-spline order. Using interpolating spline instead.');
        traj = simple_trajectory_gen(control_points, total_time, dt);
        return;
    end
    
    if ~exist('spmak','file') || ~exist('fnval','file')
        error('Curve Fitting Toolbox functions ''spmak'' and ''fnval'' are required for B-spline generation.');
    end

    control_points_t = control_points';
    knot_sequence = linspace(0, 1, n_control_points - b_spline_order + 2);
    knots = [zeros(1, b_spline_order - 1), knot_sequence, ones(1, b_spline_order - 1)];
    b_spline = spmak(knots, control_points_t);

    num_samples = floor(total_time / dt) + 1;
    t_vec = linspace(0, total_time, num_samples)';
    u_eval = linspace(0, 1, num_samples);
    traj_points_t = fnval(b_spline, u_eval);

    traj = [t_vec, traj_points_t'];
end

% Original simple_trajectory_gen function
function traj = simple_trajectory_gen(waypoints, total_time, dt)
    n_waypoints = size(waypoints, 1);
    dims = size(waypoints, 2);

    if n_waypoints == 0
        traj = zeros(0, dims + 1); % No waypoints, empty trajectory
        return;
    end
    
    t_out = (0:dt:total_time)';
    num_samples_out = length(t_out);

    if n_waypoints == 1
        traj_points = repmat(waypoints, num_samples_out, 1);
    else
        time_segments = linspace(0, total_time, n_waypoints);
        traj_points = zeros(num_samples_out, dims);
        for i = 1:dims
            traj_points(:,i) = spline(time_segments, waypoints(:,i), t_out);
        end
    end
    
    traj = [t_out, traj_points];
end

% --- Helper functions ---
function [collided_indices, has_collision] = check_trajectory_collisions(trajectory_points, map_data, world2idx_func, map_Ny, map_Nx, map_Nz)
    collided_indices = [];
    has_collision = false;
    
    for k = 1:size(trajectory_points, 1)
        point_world = trajectory_points(k, :);
        idx_float = world2idx_func(point_world);
        idx_rounded = round(idx_float);
        
        r = idx_rounded(1);
        c = idx_rounded(2);
        l = idx_rounded(3);
        
        if r >= 1 && r <= map_Ny && c >= 1 && c <= map_Nx && l >= 1 && l <= map_Nz
            if map_data(r, c, l) == 1 % 1 represents an obstacle/wall
                collided_indices = [collided_indices; r, c, l];
                has_collision = true;
            end
        end
    end
    
    if has_collision
        collided_indices = unique(collided_indices, 'rows');
    end
end

function denser_waypoints = generate_denser_waypoints(original_waypoints)
    % Insert intermediate points between each pair of consecutive waypoints
    denser_waypoints = original_waypoints(1, :); % Start with first waypoint
    
    for i = 1:size(original_waypoints, 1)-1
        current_point = original_waypoints(i, :);
        next_point = original_waypoints(i+1, :);
        
        % Add multiple intermediate points
        num_intermediates = 3; % Increase density
        for j = 1:num_intermediates
            alpha = j / (num_intermediates + 1);
            intermediate_point = current_point + alpha * (next_point - current_point);
            denser_waypoints = [denser_waypoints; intermediate_point];
        end
        
        % Add the next waypoint
        denser_waypoints = [denser_waypoints; next_point];
    end
end

function safe_waypoints = generate_obstacle_avoiding_waypoints(original_waypoints, map_data, world2idx_func, idx2world_func, map_Ny, map_Nx, map_Nz)
    % Generate waypoints that strategically avoid known obstacle areas
    safe_waypoints = [];
    
    for i = 1:size(original_waypoints, 1)
        current_wp = original_waypoints(i, :);
        safe_waypoints = [safe_waypoints; current_wp];
        
        if i < size(original_waypoints, 1)
            next_wp = original_waypoints(i+1, :);
            
            % Check if straight line between waypoints passes near obstacles
            num_checks = 20;
            for j = 1:num_checks
                alpha = j / (num_checks + 1);
                test_point = current_wp + alpha * (next_wp - current_wp);
                
                % Check surrounding area for obstacles
                idx_test = round(world2idx_func(test_point));
                r = idx_test(1); c = idx_test(2); l = idx_test(3);
                
                near_obstacle = false;
                for dr = -1:1
                    for dc = -1:1
                        check_r = r + dr;
                        check_c = c + dc;
                        if check_r >= 1 && check_r <= map_Ny && check_c >= 1 && check_c <= map_Nx && l >= 1 && l <= map_Nz
                            if map_data(check_r, check_c, l) == 1
                                near_obstacle = true;
                                break;
                            end
                        end
                    end
                    if near_obstacle, break; end
                end
                
                if near_obstacle
                    % Deflect perpendicular to the path
                    direction = next_wp - current_wp;
                    direction = direction / norm(direction);
                    perp = [-direction(2), direction(1), 0];
                    
                    for deflection_sign = [-1, 1]
                        deflected_point = test_point + deflection_sign * 0.5 * perp;
                        idx_deflected = round(world2idx_func(deflected_point));
                        rd = idx_deflected(1); cd = idx_deflected(2); ld = idx_deflected(3);
                        
                        if rd >= 1 && rd <= map_Ny && cd >= 1 && cd <= map_Nx && ld >= 1 && ld <= map_Nz
                            if map_data(rd, cd, ld) == 0
                                safe_waypoints = [safe_waypoints; deflected_point];
                                break;
                            end
                        end
                    end
                    break;
                end
            end
        end
    end
end

%% DRONE & ENVIRONMENT PARAMETERS
g                 = 9.80665;
mass_drone        = 0.68;
mass_rod          = 0.0;
mass_tip          = 0.0;
mass_total        = mass_drone + mass_rod + mass_tip;

stiffness_rod     = 100;
stiffness_wall    = 100;
critical_damping_rod  = 2*sqrt(mass_total*stiffness_rod);
critical_damping_wall = 2*sqrt(mass_total*stiffness_wall);
inertia_xx = 0.007;  inertia_yy = 0.007;  inertia_zz = 0.012;
arm_length = 0.17;   rotor_offset_top = 0.01;
motor_constant  = 8.54858e-06;  moment_constant = 0.016;
max_rot_velocity = 838;

allocation_matrix = [ 1 1 1 1;
                      0  arm_length  0 -arm_length;
                     -arm_length 0  arm_length 0;
                     -moment_constant moment_constant ...
                     -moment_constant moment_constant];

mix_matrix      = inv(motor_constant * allocation_matrix);
air_density     = 1.2041;
drag_coefficient = 0.47;
reference_area   = pi * (75e-3)^2;
