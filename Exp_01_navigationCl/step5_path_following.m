%% step5_path_following.m
% LEARNING OBJECTIVE: Implement Pure Pursuit path following controller

clc; clear; close all;

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   STEP 5: PURE PURSUIT PATH FOLLOWING\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% Load Data from Previous Steps
if ~exist('navigation_data.mat', 'file')
    error('Run Steps 3-4 first!');
end
load('navigation_data.mat');

if ~exist('planned_path', 'var') || isempty(planned_path)
    error('No path found. Run Step 4 first!');
end

fprintf('Loaded path with %d waypoints.\n', size(planned_path, 1));
fprintf('Start: (%.1f, %.1f)\n', start_position);
fprintf('Goal:  (%.1f, %.1f)\n\n', goal_position);

%% Robot Parameters
robot.wheel_radius = 0.05;      % meters
robot.wheel_base = 0.3;         % meters
robot.max_linear_vel = 0.8;     % m/s
robot.max_angular_vel = 2.5;    % rad/s

fprintf('Robot Parameters:\n');
fprintf('  Wheel radius:     %.2f m\n', robot.wheel_radius);
fprintf('  Wheel base:       %.2f m\n', robot.wheel_base);
fprintf('  Max linear vel:   %.2f m/s\n', robot.max_linear_vel);
fprintf('  Max angular vel:  %.2f rad/s\n\n', robot.max_angular_vel);

%% Pure Pursuit Controller Parameters
% STUDENTS CAN MODIFY THESE VALUES TO SEE EFFECTS
pursuit.lookahead = 0.8;        % Lookahead distance (m) - TRY: 0.3, 0.5, 0.8, 1.2
pursuit.desired_vel = 0.5;      % Desired linear velocity (m/s)

fprintf('Pure Pursuit Parameters:\n');
fprintf('  Lookahead distance: %.2f m\n', pursuit.lookahead);
fprintf('  Desired velocity:   %.2f m/s\n\n', pursuit.desired_vel);

%% Initialize Simulation
dt = 0.02;              % Time step (20 ms)
t_max = 150;            % Maximum simulation time
t = 0:dt:t_max;
N = length(t);

% Robot initial state [x, y, theta]
initial_theta = atan2(planned_path(2,2) - planned_path(1,2), ...
                      planned_path(2,1) - planned_path(1,1));
robot_state = [start_position(1), start_position(2), initial_theta];

% Storage arrays
trajectory = zeros(N, 3);
trajectory(1,:) = robot_state;

velocity_cmds = zeros(N, 2);        % [v, omega]
lookahead_points = zeros(N, 2);     % Lookahead point at each step
cross_track_error = zeros(N, 1);    % Distance from path

%% Create Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = planned_path;
controller.DesiredLinearVelocity = pursuit.desired_vel;
controller.MaxAngularVelocity = robot.max_angular_vel;
controller.LookaheadDistance = pursuit.lookahead;

%% Setup Visualization
figure('Name', 'Step 5: Pure Pursuit Path Following', 'Position', [50 50 1400 700]);

fprintf('Starting simulation...\n');
fprintf('─────────────────────────────────────────\n');

%% Main Simulation Loop
goal_reached = false;
final_step = N;

for k = 1:N-1
    % Current robot pose
    pose = trajectory(k,:);
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    
    %% ═════════════════════════
    %  PURE PURSUIT ALGORITHM
    %% ═════════════════════════
    
    % Get velocity commands from controller
    [v, omega] = controller(pose);
    
    % Apply velocity saturation
    v = max(0, min(v, robot.max_linear_vel));
    omega = max(-robot.max_angular_vel, min(omega, robot.max_angular_vel));
    
    % Store velocity commands
    velocity_cmds(k,:) = [v, omega];
    
    %% Find Lookahead Point (for visualization)
    % Calculate distance to all waypoints
    distances = sqrt((planned_path(:,1) - x).^2 + (planned_path(:,2) - y).^2);
    [min_dist, closest_idx] = min(distances);
    
    % Store cross-track error
    cross_track_error(k) = min_dist;
    
    % Find lookahead point
    lookahead_idx = closest_idx;
    for i = closest_idx:size(planned_path, 1)
        if distances(i) >= pursuit.lookahead
            lookahead_idx = i;
            break;
        end
        lookahead_idx = i;  % Use last point if no point is far enough
    end
    lookahead_points(k,:) = planned_path(lookahead_idx,:);
    
    %% ═══════════════════════════════════════════════════════════
    %  ROBOT KINEMATICS (Differential Drive)
    %% ═══════════════════════════════════════════════════════════
    
    % State derivatives
    x_dot = v * cos(theta);
    y_dot = v * sin(theta);
    theta_dot = omega;
    
    % Euler integration - update state
    x_new = x + x_dot * dt;
    y_new = y + y_dot * dt;
    theta_new = theta + theta_dot * dt;
    
    % Normalize theta to [-pi, pi]
    theta_new = wrapToPi(theta_new);
    
    % Store new state
    trajectory(k+1,:) = [x_new, y_new, theta_new];
    
    %% Check Goal Reached
    dist_to_goal = norm([x_new, y_new] - goal_position);
    if dist_to_goal < 0.3
        fprintf('  ✓ GOAL REACHED at t = %.2f s!\n', t(k));
        goal_reached = true;
        final_step = k + 1;
        break;
    end
    
    %% Check if robot is stuck or lost
    if k > 100 && norm(trajectory(k,1:2) - trajectory(k-100,1:2)) < 0.1
        fprintf('  ✗ Robot appears stuck at t = %.2f s\n', t(k));
        final_step = k;
        break;
    end
    
    %% ═══════════════════════════════════════════════════════════
    %  REAL-TIME VISUALIZATION (every 25 steps)
    %% ═══════════════════════════════════════════════════════════
    if mod(k, 25) == 0 || k == 1
        
        % --- Subplot 1: Map and Trajectory ---
        subplot(2, 3, [1, 4]);
        cla;
        show(map); hold on;
        
        % Reference path (blue dashed)
        plot(planned_path(:,1), planned_path(:,2), 'b--', 'LineWidth', 2);
        
        % Actual trajectory (green solid)
        plot(trajectory(1:k,1), trajectory(1:k,2), 'g-', 'LineWidth', 2.5);
        
        % Lookahead line and point
        plot([x, lookahead_points(k,1)], [y, lookahead_points(k,2)], ...
             'm--', 'LineWidth', 1.5);
        plot(lookahead_points(k,1), lookahead_points(k,2), 'mo', ...
             'MarkerSize', 12, 'MarkerFaceColor', 'm');
        
        % Draw robot
        drawRobot(x, y, theta, robot.wheel_base);
        
        % Start and Goal markers
        plot(start_position(1), start_position(2), 'go', ...
             'MarkerSize', 14, 'MarkerFaceColor', 'g', 'LineWidth', 2);
        plot(goal_position(1), goal_position(2), 'rp', ...
             'MarkerSize', 18, 'MarkerFaceColor', 'r', 'LineWidth', 2);
        
        title(sprintf('Navigation (t = %.1f s, dist to goal = %.2f m)', ...
              t(k), dist_to_goal), 'FontSize', 12);
        legend('Reference Path', 'Actual Path', 'Lookahead Line', ...
               'Lookahead Point', 'Robot', 'Start', 'Goal', ...
               'Location', 'northwest', 'FontSize', 8);
        
        % --- Subplot 2: Linear Velocity ---
        subplot(2, 3, 2);
        plot(t(1:k), velocity_cmds(1:k,1), 'b-', 'LineWidth', 1.5);
        hold off;
        xlabel('Time (s)'); 
        ylabel('Linear Velocity (m/s)');
        title('Linear Velocity Command', 'FontSize', 11);
        grid on;
        xlim([0, max(t(k)+5, 10)]);
        ylim([0, robot.max_linear_vel * 1.1]);
        
        % --- Subplot 3: Angular Velocity ---
        subplot(2, 3, 3);
        plot(t(1:k), velocity_cmds(1:k,2), 'r-', 'LineWidth', 1.5);
        hold off;
        xlabel('Time (s)'); 
        ylabel('Angular Velocity (rad/s)');
        title('Angular Velocity Command', 'FontSize', 11);
        grid on;
        xlim([0, max(t(k)+5, 10)]);
        ylim([-robot.max_angular_vel * 1.1, robot.max_angular_vel * 1.1]);
        
        % --- Subplot 4: Cross-Track Error ---
        subplot(2, 3, 5);
        plot(t(1:k), cross_track_error(1:k) * 100, 'm-', 'LineWidth', 1.5);
        hold off;
        xlabel('Time (s)'); 
        ylabel('Cross-Track Error (cm)');
        title(sprintf('Path Following Error (Mean: %.1f cm)', ...
              mean(cross_track_error(1:k))*100), 'FontSize', 11);
        grid on;
        xlim([0, max(t(k)+5, 10)]);
        
        % --- Subplot 5: Robot Heading ---
        subplot(2, 3, 6);
        plot(t(1:k), rad2deg(trajectory(1:k,3)), 'k-', 'LineWidth', 1.5);
        hold off;
        xlabel('Time (s)'); 
        ylabel('Heading (degrees)');
        title('Robot Heading', 'FontSize', 11);
        grid on;
        xlim([0, max(t(k)+5, 10)]);
        
        sgtitle('STEP 5: Pure Pursuit Path Following', ...
                'FontSize', 14, 'FontWeight', 'bold');
        
        drawnow;
    end
end

%% Trim Data to Actual Simulation Length
trajectory = trajectory(1:final_step,:);
velocity_cmds = velocity_cmds(1:final_step,:);
cross_track_error = cross_track_error(1:final_step);
lookahead_points = lookahead_points(1:final_step,:);
time_data = t(1:final_step)';

%% Calculate Final Metrics
total_time = time_data(end);
total_distance = sum(sqrt(diff(trajectory(:,1)).^2 + diff(trajectory(:,2)).^2));
mean_cte = mean(cross_track_error);
max_cte = max(cross_track_error);
final_error = norm(trajectory(end,1:2) - goal_position);

%% Display Results
fprintf('─────────────────────────────────────────\n');
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('SIMULATION RESULTS:\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  Goal Reached:           %s\n', mat2str(goal_reached));
fprintf('  Total Time:             %.2f s\n', total_time);
fprintf('  Total Distance:         %.2f m\n', total_distance);
fprintf('  Mean Cross-Track Error: %.2f cm\n', mean_cte * 100);
fprintf('  Max Cross-Track Error:  %.2f cm\n', max_cte * 100);
fprintf('  Final Position Error:   %.2f cm\n', final_error * 100);
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% Final Static Plot
figure('Name', 'Step 5: Final Results', 'Position', [100 100 1000 500]);

% Trajectory comparison
subplot(1, 2, 1);
show(map); hold on;
plot(planned_path(:,1), planned_path(:,2), 'b--', 'LineWidth', 2);
plot(trajectory(:,1), trajectory(:,2), 'g-', 'LineWidth', 2.5);
plot(start_position(1), start_position(2), 'go', 'MarkerSize', 14, 'MarkerFaceColor', 'g');
plot(goal_position(1), goal_position(2), 'rp', 'MarkerSize', 18, 'MarkerFaceColor', 'r');
title('Final Trajectory Comparison', 'FontSize', 12);
legend('Planned Path', 'Actual Path', 'Start', 'Goal', 'Location', 'northwest');

% Error distribution
subplot(1, 2, 2);
histogram(cross_track_error * 100, 30, 'FaceColor', [0.3 0.7 0.4], 'EdgeColor', 'k');
xlabel('Cross-Track Error (cm)', 'FontSize', 11);
ylabel('Frequency', 'FontSize', 11);
title(sprintf('Error Distribution\nMean: %.1f cm, Max: %.1f cm', ...
      mean_cte*100, max_cte*100), 'FontSize', 12);
grid on;

sgtitle('STEP 5: Path Following Results', 'FontSize', 14, 'FontWeight', 'bold');

%% Save Data for Next Step
save('navigation_data.mat', 'trajectory', 'velocity_cmds', 'time_data', ...
     'cross_track_error', 'pursuit', 'robot', 'goal_reached', '-append');

fprintf('Data saved to: navigation_data.mat\n');
fprintf('Proceed to Step 6: Simulink Integration\n');
fprintf('═══════════════════════════════════════════════════════════\n');

%% ═══════════════════════════════════════════════════════════
%  HELPER FUNCTIONS
%% ═══════════════════════════════════════════════════════════

function drawRobot(x, y, theta, wheel_base)
    % Draw robot body (circle)
    radius = wheel_base / 2;
    angles = linspace(0, 2*pi, 30);
    body_x = x + radius * cos(angles);
    body_y = y + radius * sin(angles);
    fill(body_x, body_y, [0.2 0.6 0.9], 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Draw heading arrow
    arrow_length = radius * 1.5;
    quiver(x, y, arrow_length*cos(theta), arrow_length*sin(theta), 0, ...
           'Color', 'r', 'LineWidth', 2.5, 'MaxHeadSize', 1.5);
    
    % Draw wheels
    wheel_length = radius * 0.6;
    wheel_width = radius * 0.2;
    
    % Left wheel position
    left_wheel_x = x - (wheel_base/2) * sin(theta);
    left_wheel_y = y + (wheel_base/2) * cos(theta);
    
    % Right wheel position
    right_wheel_x = x + (wheel_base/2) * sin(theta);
    right_wheel_y = y - (wheel_base/2) * cos(theta);
    
    % Draw wheels as rectangles
    drawWheel(left_wheel_x, left_wheel_y, theta, wheel_length, wheel_width);
    drawWheel(right_wheel_x, right_wheel_y, theta, wheel_length, wheel_width);
end

function drawWheel(x, y, theta, length, width)
    % Wheel corners relative to center
    corners = [
        -length/2, -width/2;
        length/2, -width/2;
        length/2, width/2;
        -length/2, width/2;
    ];
    
    % Rotation matrix
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    
    % Rotate and translate
    rotated = (R * corners')';
    wheel_x = rotated(:,1) + x;
    wheel_y = rotated(:,2) + y;
    
    fill(wheel_x, wheel_y, 'k');
end