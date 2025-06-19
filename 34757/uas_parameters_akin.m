%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% UAS – Maze mission main script (with spline smoothing)
% (c) 2024  — MIT Licence
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIALIZATION
clear; close all; clc

% Folder that holds your maze-definition *.m files
MAZE_DIR = '/home/akin/uav_course/34757/maze';
addpath(MAZE_DIR);

% Load one of the predefined 3-D mazes — this sets variable `map`
maze_1_3D          % ↩️  edit if you want a different maze

%% 0️⃣  Utility lambdas
Ny = size(map,1);                    % number of rows in the maze

% map-index  ⟷  Gazebo-world transforms (cell centres)
idx2world = @(idx) [ (idx(:,2)-1.0) ,  (Ny-idx(:,1)+0.0) ,  idx(:,3) ];
world2idx = @(pos) [  Ny-pos(:,2)   ,  pos(:,1)+1        ,  pos(:,3) ];

%% 1️⃣  Define start / goal in 0-based world coordinates
start_pos_w = [0 0 1];
goal_pos_w  = [3 5 1];

%% 2️⃣  Convert to map indices
start_idx = world2idx(start_pos_w);
goal_idx  = world2idx(goal_pos_w);

%% 3️⃣  Choose path-planner and generate the route (index space)
use_astar = true;  % set false to fall back on greedy_3d

if use_astar
    route_idx = astar_3d(map, start_idx, goal_idx);
else
    route_idx = greedy_3d(map, start_idx, goal_idx);
end

%% 4️⃣  Convert back to Gazebo world coords
route_world = idx2world(route_idx);     % centre of each cell
route_discrete = route_world;

% ──────────────────────────────────────────────────────
% ✨ Smooth route with cubic spline interpolation
points_per_segment = 10;
t_orig = 1:size(route_discrete, 1);
t_dense = linspace(1, size(route_discrete,1), ...
                   (size(route_discrete,1)-1)*points_per_segment + 1);
x_spline = spline(t_orig, route_discrete(:,1), t_dense);
y_spline = spline(t_orig, route_discrete(:,2), t_dense);
z_spline = spline(t_orig, route_discrete(:,3), t_dense);
route_dense = [x_spline(:), y_spline(:), z_spline(:)];

% Replace route with smooth version
route = route_dense;

% ──────────────────────────────────────────────────────

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

% Start position
x0 = route(1,1);
y0 = route(1,2);
z0 = route(1,3);

% Timing constants required by Simulink
sample_time  = 4e-2;       % 40 ms  ≙ 25 Hz loop rate
publish_rate = sample_time;

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

%route = [0 0 1; 1 0 1];
%route = [0 0 1; 3 0 1];
%route = [0 0 1; 9 0 1];
