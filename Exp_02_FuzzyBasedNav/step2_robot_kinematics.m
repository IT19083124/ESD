%% step2_robot_kinematics.m
% LEARNING OBJECTIVE: Understand how differential drive robots move

clc; clear; close all;

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   STEP 2: DIFFERENTIAL DRIVE ROBOT KINEMATICS\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% Robot Parameters (Explain each to students)
robot.wheel_radius = 0.05;    % 5 cm wheels
robot.wheel_base = 0.3;       % 30 cm between wheels
robot.max_speed = 10;         % max wheel angular velocity (rad/s)

fprintf('Robot Parameters:\n');
fprintf('  Wheel Radius (r): %.2f m\n', robot.wheel_radius);
fprintf('  Wheel Base (L):   %.2f m\n', robot.wheel_base);
fprintf('  Max Wheel Speed:  %.1f rad/s\n\n', robot.max_speed);

%% Simulation Setup
dt = 0.01;          % Time step (10 ms)
t_final = 15;       % Simulation duration
t = 0:dt:t_final;   % Time vector
N = length(t);      % Number of time steps

% Initialize state: [x, y, theta]
state = zeros(N, 3);
state(1,:) = [0, 0, 0];  % Start at origin, facing +X

%% Define Motion Pattern (Students can modify this!)
fprintf('Select motion pattern:\n');
fprintf('  1 - Straight Line\n');
fprintf('  2 - Circle (Left Turn)\n');
fprintf('  3 - Circle (Right Turn)\n');
fprintf('  4 - S-Curve\n');
fprintf('  5 - Square Path\n\n');

pattern = 4;  % Change this value (1-5)
fprintf('Using Pattern %d\n\n', pattern);

% Generate wheel velocities based on pattern
omega_L = zeros(N, 1);  % Left wheel angular velocity
omega_R = zeros(N, 1);  % Right wheel angular velocity

switch pattern
    case 1  % Straight line
        omega_L(:) = 5;
        omega_R(:) = 5;
        
    case 2  % Circle (left turn)
        omega_L(:) = 3;
        omega_R(:) = 6;
        
    case 3  % Circle (right turn)
        omega_L(:) = 6;
        omega_R(:) = 3;
        
    case 4  % S-Curve
        omega_L = 5 + 2*sin(0.5*t');
        omega_R = 5 - 2*sin(0.5*t');
        
    case 5  % Square (approximate)
        segment_time = t_final / 8;
        for i = 1:N
            segment = floor(t(i) / segment_time);
            if mod(segment, 2) == 0  % Straight
                omega_L(i) = 5;
                omega_R(i) = 5;
            else  % Turn
                omega_L(i) = -3;
                omega_R(i) = 3;
            end
        end
end

%% Main Simulation Loop
fprintf('Running simulation...\n');

velocity_log = zeros(N, 2);  % [v, omega] log

for k = 1:N-1
    % Current state
    x = state(k, 1);
    y = state(k, 2);
    theta = state(k, 3);
    
    % ═══════════════════════════════════════════════════════════
    % KINEMATIC EQUATIONS (Core learning content)
    % ═══════════════════════════════════════════════════════════
    
    % Convert wheel angular velocities to linear velocities
    v_L = robot.wheel_radius * omega_L(k);  % Left wheel linear velocity
    v_R = robot.wheel_radius * omega_R(k);  % Right wheel linear velocity
    
    % Robot velocities (at center point)
    v = (v_R + v_L) / 2;                    % Linear velocity
    omega = (v_R - v_L) / robot.wheel_base; % Angular velocity
    
    velocity_log(k,:) = [v, omega];
    
    % State derivatives (how position changes)
    x_dot = v * cos(theta);
    y_dot = v * sin(theta);
    theta_dot = omega;
    
    % Euler integration (update state)
    state(k+1, 1) = x + x_dot * dt;
    state(k+1, 2) = y + y_dot * dt;
    state(k+1, 3) = theta + theta_dot * dt;
end

fprintf('Simulation complete!\n\n');

%% Visualization
figure('Name', 'Step 2: Robot Kinematics', 'Position', [100 100 1200 500]);

% Plot 1: Trajectory
subplot(1, 2, 1);
plot(state(:,1), state(:,2), 'b-', 'LineWidth', 2);
hold on;

% Draw robot at intervals
draw_interval = floor(N/15);
for k = 1:draw_interval:N
    drawRobot(state(k,1), state(k,2), state(k,3), robot.wheel_base/2, [0.2 0.6 1]);
end

% Mark start and end
plot(state(1,1), state(1,2), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot(state(end,1), state(end,2), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r');

xlabel('X Position (m)', 'FontSize', 12);
ylabel('Y Position (m)', 'FontSize', 12);
title('Robot Trajectory', 'FontSize', 14);
axis equal; grid on;
legend('Path', 'Robot Pose', 'Start', 'End', 'Location', 'best');

% Plot 2: Velocities over time
subplot(1, 2, 2);
yyaxis left
plot(t, velocity_log(:,1), 'b-', 'LineWidth', 1.5);
ylabel('Linear Velocity v (m/s)', 'FontSize', 12);
ylim([min(velocity_log(:,1))-0.1, max(velocity_log(:,1))+0.1]);

yyaxis right
plot(t, velocity_log(:,2), 'r-', 'LineWidth', 1.5);
ylabel('Angular Velocity ω (rad/s)', 'FontSize', 12);

xlabel('Time (s)', 'FontSize', 12);
title('Velocity Commands', 'FontSize', 14);
grid on;
legend('Linear v', 'Angular ω', 'Location', 'best');

sgtitle('STEP 2: Differential Drive Kinematics', 'FontSize', 16, 'FontWeight', 'bold');

%% Display Key Equations
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('KEY EQUATIONS:\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  v = r(ωR + ωL)/2      Linear velocity\n');
fprintf('  ω = r(ωR - ωL)/L      Angular velocity\n');
fprintf('  ẋ = v·cos(θ)         X velocity\n');
fprintf('  ẏ = v·sin(θ)         Y velocity\n');
fprintf('  θ̇ = ω                Heading rate\n');
fprintf('═══════════════════════════════════════════════════════════\n');

%% Helper Function
function drawRobot(x, y, theta, size, color)
    % Draw triangle representing robot
    angles = theta + [0, 2.5, -2.5];
    px = x + size * cos(angles);
    py = y + size * sin(angles);
    fill(px, py, color, 'FaceAlpha', 0.7, 'EdgeColor', 'k');
end