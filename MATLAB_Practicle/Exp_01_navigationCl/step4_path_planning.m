%% step4_path_planning.m
% LEARNING OBJECTIVE: Understand and compare path planning algorithms

clc; clear; close all;

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   STEP 4: PATH PLANNING ALGORITHMS\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% Load Environment
if ~exist('navigation_data.mat', 'file')
    error('Run Step 3 first to create the environment!');
end
load('navigation_data.mat');
fprintf('Environment loaded successfully.\n\n');

%% Algorithm Selection
fprintf('Available Algorithms:\n');
fprintf('  1 - A* (Grid-based, optimal)\n');
fprintf('  2 - PRM (Probabilistic Roadmap)\n');
fprintf('  3 - RRT (Rapidly-exploring Random Tree)\n');
fprintf('  4 - Compare All\n\n');

algorithm_choice = 4;  % <-- STUDENTS: Change this (1-4)
fprintf('Selected: Option %d\n\n', algorithm_choice);

%% Setup figure
figure('Name', 'Step 4: Path Planning', 'Position', [50 50 1400 800]);

%% Initialize variables for comparison
path_astar = [];
path_prm = [];
path_rrt = [];
time_astar = NaN;
time_prm = NaN;
time_rrt = NaN;

%% ═══════════════════════════════════════════════════════════
%  A* PATH PLANNING
%% ═══════════════════════════════════════════════════════════
if algorithm_choice == 1 || algorithm_choice == 4
    fprintf('─────────────────────────────────────────\n');
    fprintf('A* Algorithm:\n');
    fprintf('─────────────────────────────────────────\n');
    
    tic;
    
    try
        % Create A* planner using the inflated map
        planner_astar = plannerAStarGrid(inflated_map);
        
        % Set diagonal connections for smoother paths
        planner_astar.DiagonalSearch = 'on';
        
        % Plan path (convert world to grid coordinates)
        start_grid = world2grid(inflated_map, start_position);
        goal_grid = world2grid(inflated_map, goal_position);
        
        % Plan in grid coordinates
        path_grid = plan(planner_astar, start_grid, goal_grid);
        
        % Convert back to world coordinates
        path_astar = grid2world(inflated_map, path_grid);
        
        time_astar = toc;
        
        fprintf('  ✓ Path found!\n');
        fprintf('  Waypoints: %d\n', size(path_astar, 1));
        fprintf('  Computation time: %.4f s\n', time_astar);
        fprintf('  Path length: %.2f m\n\n', calculatePathLength(path_astar));
        
    catch ME
        time_astar = toc;
        fprintf('  ✗ A* failed: %s\n\n', ME.message);
        path_astar = [];
    end
    
    % Plot A* result
    if algorithm_choice == 4
        subplot(2, 2, 1);
    else
        subplot(1, 1, 1);
    end
    
    show(map); hold on;
    if ~isempty(path_astar)
        plot(path_astar(:,1), path_astar(:,2), 'g-', 'LineWidth', 3);
    end
    plot(start_position(1), start_position(2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot(goal_position(1), goal_position(2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
    title(sprintf('A* Algorithm (%.4f s)', time_astar), 'FontSize', 12);
    legend('Path', 'Start', 'Goal', 'Location', 'northwest');
end

%% ═══════════════════════════════════════════════════════════
%  PRM PATH PLANNING
%% ═══════════════════════════════════════════════════════════
if algorithm_choice == 2 || algorithm_choice == 4
    fprintf('─────────────────────────────────────────\n');
    fprintf('PRM Algorithm:\n');
    fprintf('─────────────────────────────────────────\n');
    
    tic;
    
    try
        % Create PRM planner with specified number of nodes
        num_nodes = 300;
        planner_prm = mobileRobotPRM(inflated_map, num_nodes);
        planner_prm.ConnectionDistance = 3;  % Max connection distance
        
        % Plan path
        path_prm = findpath(planner_prm, start_position, goal_position);
        
        time_prm = toc;
        
        if ~isempty(path_prm)
            fprintf('  ✓ Path found!\n');
            fprintf('  Nodes sampled: %d\n', num_nodes);
            fprintf('  Path waypoints: %d\n', size(path_prm, 1));
            fprintf('  Computation time: %.4f s\n', time_prm);
            fprintf('  Path length: %.2f m\n\n', calculatePathLength(path_prm));
        else
            fprintf('  ✗ No path found. Try increasing nodes.\n\n');
        end
        
    catch ME
        time_prm = toc;
        fprintf('  ✗ PRM failed: %s\n\n', ME.message);
        path_prm = [];
    end
    
    % Plot PRM result
    if algorithm_choice == 4
        subplot(2, 2, 2);
    elseif algorithm_choice == 2
        subplot(1, 1, 1);
    end
    
    show(planner_prm); hold on;
    if ~isempty(path_prm)
        plot(path_prm(:,1), path_prm(:,2), 'g-', 'LineWidth', 3);
    end
    plot(start_position(1), start_position(2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot(goal_position(1), goal_position(2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
    title(sprintf('PRM (%d nodes, %.4f s)', num_nodes, time_prm), 'FontSize', 12);
end

%% ═══════════════════════════════════════════════════════════
%  RRT PATH PLANNING
%% ═══════════════════════════════════════════════════════════
if algorithm_choice == 3 || algorithm_choice == 4
    fprintf('─────────────────────────────────────────\n');
    fprintf('RRT Algorithm:\n');
    fprintf('─────────────────────────────────────────\n');
    
    tic;
    
    try
        % Create state space for SE2 (x, y, theta)
        ss = stateSpaceSE2;
        ss.StateBounds = [0 map_width; 0 map_height; -pi pi];
        
        % Create state validator using the inflated map
        sv = validatorOccupancyMap(ss);
        sv.Map = inflated_map;
        sv.ValidationDistance = 0.1;
        
        % Create RRT planner
        planner_rrt = plannerRRT(ss, sv);
        planner_rrt.MaxConnectionDistance = 2.0;
        planner_rrt.MaxIterations = 5000;
        planner_rrt.GoalBias = 0.1;
        planner_rrt.GoalReachedFcn = @(~, state, goal) ...
            norm(state(1:2) - goal(1:2)) < 0.5;
        
        % Define start and goal with orientation
        start_pose = [start_position, 0];
        goal_pose = [goal_position, 0];
        
        % Plan path (set random seed for reproducibility)
        rng(42);
        [path_rrt_full, info_rrt] = plan(planner_rrt, start_pose, goal_pose);
        
        time_rrt = toc;
        
        if ~isempty(path_rrt_full)
            path_rrt = path_rrt_full(:, 1:2);  % Extract x, y only
            
            fprintf('  ✓ Path found!\n');
            fprintf('  Tree nodes explored: %d\n', size(info_rrt.TreeData, 1));
            fprintf('  Path waypoints: %d\n', size(path_rrt, 1));
            fprintf('  Computation time: %.4f s\n', time_rrt);
            fprintf('  Path length: %.2f m\n\n', calculatePathLength(path_rrt));
        else
            fprintf('  ✗ No path found.\n\n');
            path_rrt = [];
        end
        
    catch ME
        time_rrt = toc;
        fprintf('  ✗ RRT failed: %s\n\n', ME.message);
        path_rrt = [];
        info_rrt = struct('TreeData', []);
    end
    
    % Plot RRT result
    if algorithm_choice == 4
        subplot(2, 2, 3);
    elseif algorithm_choice == 3
        subplot(1, 1, 1);
    end
    
    show(map); hold on;
    
    % Draw RRT tree (exploration visualization)
    if exist('info_rrt', 'var') && ~isempty(info_rrt.TreeData)
        tree = info_rrt.TreeData;
        for i = 1:size(tree, 1)
            if tree(i, 4) > 0  % Has parent
                parent_idx = tree(i, 4);
                plot([tree(i,1), tree(parent_idx,1)], ...
                     [tree(i,2), tree(parent_idx,2)], ...
                     'c-', 'LineWidth', 0.3);
            end
        end
    end
    
    % Plot final path
    if ~isempty(path_rrt)
        plot(path_rrt(:,1), path_rrt(:,2), 'g-', 'LineWidth', 3);
    end
    plot(start_position(1), start_position(2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot(goal_position(1), goal_position(2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
    title(sprintf('RRT Algorithm (%.4f s)', time_rrt), 'FontSize', 12);
end

%% ═══════════════════════════════════════════════════════════
%  COMPARISON SUMMARY
%% ═══════════════════════════════════════════════════════════
if algorithm_choice == 4
    subplot(2, 2, 4);
    
    % Calculate path lengths
    len_astar = calculatePathLength(path_astar);
    len_prm = calculatePathLength(path_prm);
    len_rrt = calculatePathLength(path_rrt);
    
    % Create comparison bar chart
    algorithms = categorical({'A*', 'PRM', 'RRT'});
    algorithms = reordercats(algorithms, {'A*', 'PRM', 'RRT'});
    
    times = [time_astar, time_prm, time_rrt];
    lengths = [len_astar, len_prm, len_rrt];
    
    % Normalize for visualization
    yyaxis left
    bar(algorithms, times, 'FaceColor', [0.3 0.5 0.8]);
    ylabel('Computation Time (s)', 'FontSize', 10);
    
    yyaxis right
    hold on;
    plot(algorithms, lengths, 'r-o', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    ylabel('Path Length (m)', 'FontSize', 10);
    
    title('Algorithm Comparison', 'FontSize', 12);
    grid on;
    
    % Print comparison table
    fprintf('═══════════════════════════════════════════════════════════\n');
    fprintf('COMPARISON SUMMARY:\n');
    fprintf('═══════════════════════════════════════════════════════════\n');
    fprintf('%-12s %-15s %-15s %-12s\n', 'Algorithm', 'Time (s)', 'Path Length (m)', 'Waypoints');
    fprintf('─────────────────────────────────────────────────────────\n');
    fprintf('%-12s %-15.4f %-15.2f %-12d\n', 'A*', time_astar, len_astar, size(path_astar, 1));
    fprintf('%-12s %-15.4f %-15.2f %-12d\n', 'PRM', time_prm, len_prm, size(path_prm, 1));
    fprintf('%-12s %-15.4f %-15.2f %-12d\n', 'RRT', time_rrt, len_rrt, size(path_rrt, 1));
    fprintf('═══════════════════════════════════════════════════════════\n\n');
end

%% ═══════════════════════════════════════════════════════════
%  SELECT BEST PATH AND SAVE
%% ═══════════════════════════════════════════════════════════

% Choose the best path for next step (prefer PRM for smoothness)
if ~isempty(path_prm)
    planned_path = path_prm;
    selected_algorithm = 'PRM';
elseif ~isempty(path_astar)
    planned_path = path_astar;
    selected_algorithm = 'A*';
elseif ~isempty(path_rrt)
    planned_path = path_rrt;
    selected_algorithm = 'RRT';
else
    error('No valid path found by any algorithm!');
end

fprintf('Selected path from %s algorithm for navigation.\n', selected_algorithm);
fprintf('Path has %d waypoints.\n\n', size(planned_path, 1));

% Save to file
save('navigation_data.mat', 'planned_path', 'path_astar', 'path_prm', 'path_rrt', ...
     'time_astar', 'time_prm', 'time_rrt', '-append');

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('Data saved to: navigation_data.mat\n');
fprintf('Proceed to Step 5: Path Following\n');
fprintf('═══════════════════════════════════════════════════════════\n');

sgtitle('STEP 4: Path Planning Algorithms', 'FontSize', 16, 'FontWeight', 'bold');

%% ═══════════════════════════════════════════════════════════
%  HELPER FUNCTION
%% ═══════════════════════════════════════════════════════════
function len = calculatePathLength(path)
    if isempty(path)
        len = Inf;
        return;
    end
    len = sum(sqrt(diff(path(:,1)).^2 + diff(path(:,2)).^2));
end