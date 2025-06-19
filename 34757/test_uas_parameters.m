%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% test_uas_parameters.m - Unit tests for UAS maze mission parameters
% (c) 2024 — MIT Licence
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function tests = test_uas_parameters
	tests = functiontests(localfunctions);
end

function setupOnce(testCase)
	% Add maze directory to path for testing
	MAZE_DIR = '/Users/rami/Documents/DTU/Semester 2/UAS/uav_course/34757/maze';
	addpath(MAZE_DIR);
	
	% Load test maze
	maze_1_3D;
	testCase.TestData.map = map;
	testCase.TestData.Ny = size(map,1);
	
	% Define transformation functions
	testCase.TestData.idx2world = @(idx) [ (idx(:,2)-0.5) ,  (testCase.TestData.Ny-idx(:,1)+0.5) ,  idx(:,3) ];
	testCase.TestData.world2idx = @(pos) [  testCase.TestData.Ny-pos(:,2)   ,  pos(:,1)+1        ,  pos(:,3) ];
end

function test_coordinate_transformations(testCase)
	idx2world = testCase.TestData.idx2world;
	world2idx = testCase.TestData.world2idx;
	
	% Test basic transformation
	test_idx = [5, 3, 1];
	world_pos = idx2world(test_idx);
	back_to_idx = world2idx(world_pos);
	
	verifyEqual(testCase, back_to_idx, test_idx, 'AbsTol', 1e-10, ...
		'Coordinate transformation should be reversible');
	
	% Test multiple points
	test_indices = [1, 1, 1; 5, 3, 1; 10, 10, 1];
	world_positions = idx2world(test_indices);
	back_indices = world2idx(world_positions);
	
	verifyEqual(testCase, back_indices, test_indices, 'AbsTol', 1e-10, ...
		'Multiple point transformation should be reversible');
end

function test_start_goal_positions(testCase)
	world2idx = testCase.TestData.world2idx;
	map = testCase.TestData.map;
	
	start_pos_w = [0 0 1];
	goal_pos_w = [3 5 1];
	
	start_idx = world2idx(start_pos_w);
	goal_idx = world2idx(goal_pos_w);
	
	% Verify indices are within map bounds
	verifyTrue(testCase, all(start_idx >= 1) && all(start_idx <= size(map)), ...
		'Start position should be within map bounds');
	verifyTrue(testCase, all(goal_idx >= 1) && all(goal_idx <= size(map)), ...
		'Goal position should be within map bounds');
	
	% Verify start and goal are in free space
	verifyEqual(testCase, map(start_idx(1), start_idx(2), start_idx(3)), 0, ...
		'Start position should be in free space');
	verifyEqual(testCase, map(goal_idx(1), goal_idx(2), goal_idx(3)), 0, ...
		'Goal position should be in free space');
end

function test_trajectory_generation(testCase)
	% Test the simple_trajectory_gen function
	waypoints = [0, 0, 1; 1, 1, 1; 2, 0, 1];
	total_time = 5.0;
	dt = 0.1;
	
	traj = simple_trajectory_gen(waypoints, total_time, dt);
	
	% Verify trajectory structure
	expected_length = length(0:dt:total_time);
	verifySize(testCase, traj, [expected_length, 4], ...
		'Trajectory should have correct dimensions');
	
	% Verify time column
	expected_time = (0:dt:total_time)';
	verifyEqual(testCase, traj(:,1), expected_time, 'AbsTol', 1e-10, ...
		'Time column should be correct');
	
	% Verify trajectory starts and ends at correct waypoints
	verifyEqual(testCase, traj(1, 2:4), waypoints(1,:), 'AbsTol', 1e-6, ...
		'Trajectory should start at first waypoint');
	verifyEqual(testCase, traj(end, 2:4), waypoints(end,:), 'AbsTol', 1e-6, ...
		'Trajectory should end at last waypoint');
end

function test_physical_parameters(testCase)
	% Test drone and environment parameters
	g = 9.80665;
	mass_drone = 0.68;
	mass_total = mass_drone;  % rod and tip mass are 0
	
	verifyGreaterThan(testCase, g, 0, 'Gravity should be positive');
	verifyGreaterThan(testCase, mass_drone, 0, 'Drone mass should be positive');
	verifyEqual(testCase, mass_total, mass_drone, 'Total mass calculation');
	
	% Test inertia values
	inertia_xx = 0.007; inertia_yy = 0.007; inertia_zz = 0.012;
	verifyGreaterThan(testCase, inertia_xx, 0, 'Inertia xx should be positive');
	verifyGreaterThan(testCase, inertia_yy, 0, 'Inertia yy should be positive');
	verifyGreaterThan(testCase, inertia_zz, 0, 'Inertia zz should be positive');
end

function test_allocation_matrix(testCase)
	arm_length = 0.17;
	moment_constant = 0.016;
	
	allocation_matrix = [ 1 1 1 1;
						  0  arm_length  0 -arm_length;
						 -arm_length 0  arm_length 0;
						 -moment_constant moment_constant ...
						 -moment_constant moment_constant];
	
	motor_constant = 8.54858e-06;
	mix_matrix = inv(motor_constant * allocation_matrix);
	
	% Test matrix dimensions
	verifySize(testCase, allocation_matrix, [4, 4], ...
		'Allocation matrix should be 4x4');
	verifySize(testCase, mix_matrix, [4, 4], ...
		'Mix matrix should be 4x4');
	
	% Test that mix_matrix is indeed the inverse
	identity_test = (motor_constant * allocation_matrix) * mix_matrix;
	verifyEqual(testCase, identity_test, eye(4), 'AbsTol', 1e-10, ...
		'Mix matrix should be inverse of motor_constant * allocation_matrix');
end

function test_wall_collision_detection(testCase)
	map = testCase.TestData.map;
	
	% Create a test route that goes through walls
	route_idx_with_walls = [1, 1, 1; 2, 2, 1];  % Assuming these might be walls
	
	% Check wall collision function
	onWall = arrayfun(@(k) ...
			  map(route_idx_with_walls(k,1), route_idx_with_walls(k,2), route_idx_with_walls(k,3)), ...
			  1:size(route_idx_with_walls,1));
	
	verifySize(testCase, onWall, [1, size(route_idx_with_walls,1)], ...
		'Wall collision check should return correct size array');
	verifyClass(testCase, onWall, 'logical', ...
		'Wall collision check should return logical array');
end

function test_timing_parameters(testCase)
	sample_time = 4e-2;  % 40 ms
	publish_rate = sample_time;
	
	verifyGreaterThan(testCase, sample_time, 0, ...
		'Sample time should be positive');
	verifyEqual(testCase, publish_rate, sample_time, ...
		'Publish rate should equal sample time');
	
	% Verify reasonable control loop frequency (25 Hz)
	expected_frequency = 25;  % Hz
	actual_frequency = 1 / sample_time;
	verifyEqual(testCase, actual_frequency, expected_frequency, 'AbsTol', 0.1, ...
		'Control loop should run at approximately 25 Hz');
end

function test_maze_structure(testCase)
	map = testCase.TestData.map;
	
	% Test map is 3D logical array
	verifyTrue(testCase, ndims(map) == 3, ...
		'Map should be 3-dimensional');
	verifyClass(testCase, map, 'logical', ...
		'Map should be logical array');
	
	% Test map contains both walls and free space
	verifyTrue(testCase, any(map(:)), ...
		'Map should contain some walls (true values)');
	verifyTrue(testCase, any(~map(:)), ...
		'Map should contain some free space (false values)');
end

% Helper function for trajectory generation (copied from main script)
function traj = simple_trajectory_gen(waypoints, total_time, dt)
	n_waypoints = size(waypoints, 1);
	time_segments = linspace(0, total_time, n_waypoints);
	
	t = 0:dt:total_time;
	
	x_traj = spline(time_segments, waypoints(:,1), t);
	y_traj = spline(time_segments, waypoints(:,2), t);
	z_traj = spline(time_segments, waypoints(:,3), t);
	
	traj = [t' x_traj' y_traj' z_traj'];
end