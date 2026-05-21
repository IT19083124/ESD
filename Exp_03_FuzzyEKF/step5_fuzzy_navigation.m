%% step5_fuzzy_navigation.m
% LEARNING OBJECTIVE: Real-time reactive fuzzy navigation with obstacle avoidance

clc; clear; close all;

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   STEP 5: FUZZY LOGIC REACTIVE NAVIGATION\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% Load Environment and FIS
if ~exist('navigation_data.mat', 'file')
    error('Run Step 3 first to create the environment!');
end
load('navigation_data.mat');

if ~exist('fis', 'var')
    if exist('robot_navigation_fis.fis', 'file')
        fis = readfis('robot_navigation_fis');
        fprintf('FIS loaded from file.\n');
    else
        error('Run Step 4 first to create the FIS!');
    end
end

fprintf('Environment and FIS loaded.\n\n');

%% Robot Parameters
robot.wheel_radius  = 0.05;
robot.wheel_base    = 0.3;
robot.max_lin_vel   = 0.8;
robot.max_ang_vel   = 2.5;
robot.sensor_range  = 5.0;  % Max sensor range (m)
robot.sensor_angles = [-pi/2, -pi/4, 0, pi/4, pi/2];  % 5 beam sensor angles

fprintf('Sensor configuration: %d beams\n', length(robot.sensor_angles));
fprintf('Sensor range: %.1f m\n\n', robot.sensor_range);

%% Simulation Setup
dt    = 0.05;
t_max = 200;
t     = 0:dt:t_max;
N     = length(t);

% Initial state [x, y, theta] — face toward goal
dx = goal_position(1) - start_position(1);
dy = goal_position(2) - start_position(2);
init_theta = atan2(dy, dx);

robot_state = [start_position, init_theta];

% Storage
trajectory       = zeros(N, 3);
trajectory(1,:)  = robot_state;
velocity_log     = zeros(N, 2);
sensor_log       = zeros(N, 5);   % 5 sensor readings per step
fuzzy_inputs_log = zeros(N, 3);   % [dist_front, dist_left, heading_err]

%% Setup Figure
figure('Name', 'Step 5: Fuzzy Navigation', 'Position', [50 50 1400 720]);

fprintf('Starting fuzzy navigation simulation...\n');
fprintf('─────────────────────────────────────────\n');

%% Main Simulation Loop
goal_reached = false;
final_step   = N;

for k = 1:N-1
    x     = trajectory(k, 1);
    y     = trajectory(k, 2);
    theta = trajectory(k, 3);

    %% ─── 1. SENSOR SIMULATION (Ray Casting) ─────────────────────
    sensor_dists = simulateSensors(x, y, theta, robot.sensor_angles, ...
                                   robot.sensor_range, map);
    sensor_log(k,:) = sensor_dists;

    %% ─── 2. EXTRACT FUZZY INPUTS ────────────────────────────────
    % Front distance: centre beam (index 3 for 5 beams)
    dist_front = sensor_dists(3);

    % Left distance: average of left-side beams (indices 1,2)
    dist_left  = mean(sensor_dists(1:2));

    % Heading error to goal
    angle_to_goal = atan2(goal_position(2) - y, goal_position(1) - x);
    heading_err   = wrapToPi(angle_to_goal - theta);

    % Clamp inputs to FIS ranges
    dist_front  = max(0, min(5, dist_front));
    dist_left   = max(0, min(5, dist_left));
    heading_err = max(-pi, min(pi, heading_err));

    fuzzy_inputs_log(k,:) = [dist_front, dist_left, heading_err];

    %% ─── 3. EVALUATE FUZZY RULES ────────────────────────────────
    fuzzy_output = evalfis(fis, [dist_front, dist_left, heading_err]);

    v     = fuzzy_output(1);
    omega = fuzzy_output(2);

    % Clamp outputs to robot limits
    v     = max(0, min(v, robot.max_lin_vel));
    omega = max(-robot.max_ang_vel, min(omega, robot.max_ang_vel));

    velocity_log(k,:) = [v, omega];

    %% ─── 4. INTEGRATE KINEMATICS ────────────────────────────────
    x_new     = x + v * cos(theta) * dt;
    y_new     = y + v * sin(theta) * dt;
    theta_new = wrapToPi(theta + omega * dt);

    % Boundary check (keep inside map)
    x_new = max(0.3, min(map_width  - 0.3, x_new));
    y_new = max(0.3, min(map_height - 0.3, y_new));

    trajectory(k+1,:) = [x_new, y_new, theta_new];

    %% ─── 5. GOAL CHECK ──────────────────────────────────────────
    dist_to_goal = norm([x_new, y_new] - goal_position);
    if dist_to_goal < 0.4
        fprintf('  ✓ GOAL REACHED at t = %.2f s!\n', t(k));
        goal_reached = true;
        final_step   = k + 1;
        break;
    end

    %% ─── 6. COLLISION CHECK ─────────────────────────────────────
    if checkOccupancy(map, [x_new, y_new])
        fprintf('  ✗ Collision at t = %.2f s — robot hit obstacle!\n', t(k));
        final_step = k;
        break;
    end

    %% ─── 7. REAL-TIME VISUALIZATION (every 20 steps) ────────────
    if mod(k, 20) == 0 || k == 1

        % Panel 1: Map + trajectory
        subplot(2, 3, [1, 4]);
        cla;
        show(map); hold on;
        plot(trajectory(1:k,1), trajectory(1:k,2), 'g-', 'LineWidth', 2.5);
        drawRobotFuzzy(x, y, theta, robot.wheel_base, sensor_dists, ...
                       robot.sensor_angles, robot.sensor_range);
        plot(start_position(1), start_position(2), 'go', ...
             'MarkerSize', 14, 'MarkerFaceColor', 'g', 'LineWidth', 2);
        plot(goal_position(1), goal_position(2), 'rp', ...
             'MarkerSize', 18, 'MarkerFaceColor', 'r', 'LineWidth', 2);
        % Draw goal attraction line
        plot([x, goal_position(1)], [y, goal_position(2)], ...
             'y--', 'LineWidth', 1, 'Color', [1 0.8 0]);
        title(sprintf('Fuzzy Navigation  t=%.1fs  dist=%.2fm', t(k), dist_to_goal), ...
              'FontSize', 12);
        legend({'Trajectory','Robot','Sensors','Start','Goal','To Goal'}, ...
               'Location', 'northwest', 'FontSize', 7);

        % Panel 2: Sensor readings
        subplot(2, 3, 2);
        bar(rad2deg(robot.sensor_angles), sensor_dists, 'FaceColor', [0.2 0.6 0.9]);
        xlabel('Sensor Angle (°)'); ylabel('Distance (m)');
        title(sprintf('Sensor Readings\nFront: %.2fm  Left: %.2fm', ...
              dist_front, dist_left), 'FontSize', 10);
        ylim([0, robot.sensor_range]);
        grid on;

        % Panel 3: Heading error
        subplot(2, 3, 3);
        theta_hist = trajectory(1:k, 3);
        h_err = zeros(k,1);
        for i = 1:k
            ag = atan2(goal_position(2)-trajectory(i,2), goal_position(1)-trajectory(i,1));
            h_err(i) = rad2deg(wrapToPi(ag - trajectory(i,3)));
        end
        plot(t(1:k), h_err, 'b-', 'LineWidth', 1.5); hold off;
        xlabel('Time (s)'); ylabel('Error (°)');
        title('Heading Error to Goal', 'FontSize', 10);
        grid on; xlim([0, max(t(k)+5, 10)]);

        % Panel 4: Linear velocity
        subplot(2, 3, 5);
        plot(t(1:k), velocity_log(1:k,1), 'b-', 'LineWidth', 1.5); hold off;
        xlabel('Time (s)'); ylabel('v (m/s)');
        title('Linear Velocity', 'FontSize', 10);
        grid on; ylim([0, robot.max_lin_vel*1.1]);
        xlim([0, max(t(k)+5, 10)]);

        % Panel 5: Angular velocity
        subplot(2, 3, 6);
        plot(t(1:k), velocity_log(1:k,2), 'r-', 'LineWidth', 1.5); hold off;
        xlabel('Time (s)'); ylabel('ω (rad/s)');
        title('Angular Velocity', 'FontSize', 10);
        grid on;
        ylim([-robot.max_ang_vel*1.1, robot.max_ang_vel*1.1]);
        xlim([0, max(t(k)+5, 10)]);

        sgtitle('STEP 5: Fuzzy Logic Reactive Navigation', ...
                'FontSize', 14, 'FontWeight', 'bold');
        drawnow;
    end
end

%% Trim Data
trajectory    = trajectory(1:final_step,:);
velocity_log  = velocity_log(1:final_step,:);
sensor_log    = sensor_log(1:final_step,:);
time_data     = t(1:final_step)';

%% Final Metrics
total_distance = sum(sqrt(diff(trajectory(:,1)).^2 + diff(trajectory(:,2)).^2));
total_time     = time_data(end);
final_error    = norm(trajectory(end,1:2) - goal_position);
mean_speed     = mean(velocity_log(:,1));

fprintf('─────────────────────────────────────────\n\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('FUZZY NAVIGATION RESULTS:\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  Goal Reached:      %s\n', mat2str(goal_reached));
fprintf('  Total Time:        %.2f s\n', total_time);
fprintf('  Total Distance:    %.2f m\n', total_distance);
fprintf('  Mean Speed:        %.3f m/s\n', mean_speed);
fprintf('  Final Error:       %.2f cm\n', final_error * 100);
fprintf('  Efficiency:        %.1f%%\n', ...
        (norm(goal_position - start_position) / total_distance) * 100);
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% Final Static Plot
figure('Name', 'Step 5: Final Results', 'Position', [100 100 1000 480]);

subplot(1,2,1);
show(map); hold on;
plot(trajectory(:,1), trajectory(:,2), 'g-', 'LineWidth', 2.5);
plot(start_position(1), start_position(2), 'go', 'MarkerSize', 14, 'MarkerFaceColor', 'g');
plot(goal_position(1), goal_position(2), 'rp', 'MarkerSize', 18, 'MarkerFaceColor', 'r');
title('Fuzzy Navigation Trajectory', 'FontSize', 12);
legend('Trajectory', 'Start', 'Goal', 'Location', 'northwest');

subplot(1,2,2);
plot(time_data, velocity_log(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(time_data, velocity_log(:,2), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity');
title('Velocity Commands Over Time', 'FontSize', 12);
legend('Linear v (m/s)', 'Angular ω (rad/s)');
grid on;

sgtitle('STEP 5: Fuzzy Navigation Results', 'FontSize', 14, 'FontWeight', 'bold');

save('navigation_data.mat', 'trajectory', 'velocity_log', 'time_data', ...
     'goal_reached', 'fuzzy_inputs_log', '-append');

fprintf('Data saved to navigation_data.mat\n');

%% ═══════════════════════════════════════════════════════════
%  HELPER FUNCTIONS
%% ═══════════════════════════════════════════════════════════

function dists = simulateSensors(x, y, theta, angles, max_range, map)
    % Ray-cast sensor simulation on occupancy map
    dists = max_range * ones(1, length(angles));
    step  = 0.05;  % Ray step size (m)

    for a = 1:length(angles)
        beam_angle = theta + angles(a);
        for r = step:step:max_range
            rx = x + r * cos(beam_angle);
            ry = y + r * sin(beam_angle);
            % Check map bounds
            if rx < 0 || rx >= map.XWorldLimits(2) || ...
               ry < 0 || ry >= map.YWorldLimits(2)
                dists(a) = r;
                break;
            end
            if checkOccupancy(map, [rx, ry])
                dists(a) = r;
                break;
            end
        end
    end
end

function drawRobotFuzzy(x, y, theta, wheel_base, sensor_dists, sensor_angles, max_range)
    % Draw robot body
    radius = wheel_base / 2;
    angs   = linspace(0, 2*pi, 30);
    fill(x + radius*cos(angs), y + radius*sin(angs), ...
         [0.2 0.6 0.9], 'FaceAlpha', 0.85, 'EdgeColor', 'k');

    % Heading arrow
    quiver(x, y, radius*1.5*cos(theta), radius*1.5*sin(theta), 0, ...
           'r', 'LineWidth', 2.5, 'MaxHeadSize', 1.5);

    % Draw sensor beams
    colors = {[1 0.4 0.4], [1 0.7 0.3], [0.3 1 0.3], [0.3 0.8 1], [0.7 0.4 1]};
    for i = 1:length(sensor_angles)
        beam_angle = theta + sensor_angles(i);
        bx = x + sensor_dists(i) * cos(beam_angle);
        by = y + sensor_dists(i) * sin(beam_angle);
        plot([x bx], [y by], '-', 'Color', colors{i}, 'LineWidth', 1.2);
        plot(bx, by, 'o', 'Color', colors{i}, 'MarkerSize', 5, 'MarkerFaceColor', colors{i});
    end
end