%% step3_create_environment.m (FIXED)
% LEARNING OBJECTIVE: Create and understand occupancy grid maps

clc; clear; close all;

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   STEP 3: ENVIRONMENT REPRESENTATION\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% Map Parameters
map_width = 20;      % meters
map_height = 20;     % meters
resolution = 10;     % cells per meter (higher = more detail)

fprintf('Map Parameters:\n');
fprintf('  Size: %d x %d meters\n', map_width, map_height);
fprintf('  Resolution: %d cells/meter\n', resolution);
fprintf('  Total cells: %d x %d = %d\n', ...
    map_width*resolution, map_height*resolution, ...
    map_width*resolution*map_height*resolution);
fprintf('\n');

%% Create Empty Map
map = binaryOccupancyMap(map_width, map_height, resolution);

%% Add Boundary Walls and Obstacles using World Coordinates
fprintf('Adding boundary walls...\n');

wall_thickness = 0.2;

% Create wall coordinates in WORLD frame (not grid)
% We will use setOccupancy with world coordinates

% Method: Set occupancy using world coordinate points
% Bottom wall
for x = 0:0.1:map_width
    for y = 0:0.1:wall_thickness
        setOccupancy(map, [x, y], 1);
    end
end

% Top wall
for x = 0:0.1:map_width
    for y = (map_height-wall_thickness):0.1:map_height
        setOccupancy(map, [x, y], 1);
    end
end

% Left wall
for y = 0:0.1:map_height
    for x = 0:0.1:wall_thickness
        setOccupancy(map, [x, y], 1);
    end
end

% Right wall
for y = 0:0.1:map_height
    for x = (map_width-wall_thickness):0.1:map_width
        setOccupancy(map, [x, y], 1);
    end
end

fprintf('  Walls added.\n');

%% Add Obstacles
fprintf('Adding obstacles...\n');

% Define obstacles: [x, y, width, height] in world coordinates
obstacles = [
    % Large obstacles
    4,  3,  2.5, 3;
    12, 4,  2,   4;
    8,  9,  3,   2;
    15, 10, 2,   4;
    4,  13, 3,   2;
    11, 15, 3,   2;
    % Small obstacles
    7,  3,  1,   1;
    17, 3,  1.5, 1.5;
    2,  8,  1,   2;
    18, 8,  1,   2;
];

% Add each obstacle using world coordinates
for i = 1:size(obstacles, 1)
    obs_x = obstacles(i, 1);
    obs_y = obstacles(i, 2);
    obs_w = obstacles(i, 3);
    obs_h = obstacles(i, 4);
    
    % Fill obstacle area with occupied cells
    for x = obs_x:0.1:(obs_x + obs_w)
        for y = obs_y:0.1:(obs_y + obs_h)
            if x > 0 && x < map_width && y > 0 && y < map_height
                setOccupancy(map, [x, y], 1);
            end
        end
    end
end

fprintf('  Added %d obstacles\n\n', size(obstacles, 1));

%% Create Inflated Map (for path planning)
fprintf('Creating inflated map for safe navigation...\n');
robot_radius = 0.35;  % Robot radius + safety margin

inflated_map = copy(map);
inflate(inflated_map, robot_radius);

fprintf('  Inflation radius: %.2f m\n\n', robot_radius);

%% Define Start and Goal
start_position = [2, 2];
goal_position = [18, 18];

fprintf('Navigation Points:\n');
fprintf('  Start: (%.1f, %.1f)\n', start_position);
fprintf('  Goal:  (%.1f, %.1f)\n\n', goal_position);

%% Visualization
figure('Name', 'Step 3: Environment', 'Position', [100 100 1200 500]);

% Original map
subplot(1, 2, 1);
show(map);
hold on;
plot(start_position(1), start_position(2), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot(goal_position(1), goal_position(2), 'r*', 'MarkerSize', 20, 'LineWidth', 3);
title('Original Map', 'FontSize', 14);
legend('Start', 'Goal', 'Location', 'northwest');

% Inflated map
subplot(1, 2, 2);
show(inflated_map);
hold on;
plot(start_position(1), start_position(2), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot(goal_position(1), goal_position(2), 'r*', 'MarkerSize', 20, 'LineWidth', 3);
title(sprintf('Inflated Map (radius = %.2f m)', robot_radius), 'FontSize', 14);
legend('Start', 'Goal', 'Location', 'northwest');

sgtitle('STEP 3: Environment Representation', 'FontSize', 16, 'FontWeight', 'bold');

%% Save for Next Steps
save('navigation_data.mat', 'map', 'inflated_map', 'start_position', 'goal_position', ...
     'resolution', 'map_width', 'map_height', 'robot_radius', 'obstacles');

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('Data saved to: navigation_data.mat\n');
fprintf('Proceed to Step 4: Path Planning\n');
fprintf('═══════════════════════════════════════════════════════════\n');