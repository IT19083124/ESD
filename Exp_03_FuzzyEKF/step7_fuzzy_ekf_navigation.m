%% step7_fuzzy_ekf_navigation.m
% LEARNING OBJECTIVE: Integrate EKF sensor fusion into the fuzzy navigator
%
% ARCHITECTURE:
%
%   ┌─────────────┐    ┌─────────────────────────────────────────┐
%   │   SENSORS   │    │           SENSOR FUSION LAYER           │
%   │             │    │                                         │
%   │ LiDAR beams ├───►│  EKF Predict: motion model (odom+IMU)  │
%   │ Odometry    ├───►│  EKF Update:  GPS correction            │
%   │ IMU gyro    ├───►│  Output: [x̂, ŷ, θ̂, σ_x, σ_y, σ_θ]    │
%   │ GPS fix     ├───►│                                         │
%   └─────────────┘    └──────────────────┬──────────────────────┘
%                                         │
%                                         ▼
%                       ┌─────────────────────────────────┐
%                       │      FUZZY NAVIGATOR (FIS)      │
%                       │                                 │
%                       │  Inputs: LiDAR distances +      │
%                       │          EKF heading error +    │
%                       │          EKF uncertainty weight │
%                       │  Output: v (m/s), ω (rad/s)    │
%                       └────────────────┬────────────────┘
%                                        │
%                                        ▼
%                         ┌─────────────────────────┐
%                         │   DIFFERENTIAL DRIVE    │
%                         │   Kinematic Integration │
%                         └─────────────────────────┘

clc; clear; close all;

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   STEP 7: FUZZY NAVIGATION WITH EKF SENSOR FUSION\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% ─── Load Environment and FIS ───────────────────────────────────────────
if ~exist('navigation_data.mat', 'file')
    error('Run Steps 3 and 6 first!');
end
load('navigation_data.mat');

% Load FIS (from step4_fuzzy_fis.m)
if ~exist('fis', 'var')
    if exist('robot_navigation_fis.fis', 'file')
        fis = readfis('robot_navigation_fis');
        fprintf('FIS loaded from robot_navigation_fis.fis\n');
    else
        % Build a minimal FIS inline so this step is self-contained
        fprintf('FIS not found — building inline FIS...\n');
        fis = buildInlineFIS();
        fprintf('Inline FIS built.\n');
    end
end

fprintf('Environment: %d x %d m  |  Start: (%.1f,%.1f)  |  Goal: (%.1f,%.1f)\n\n', ...
        map_width, map_height, start_position(1), start_position(2), ...
        goal_position(1), goal_position(2));

%% ─── Robot Parameters ────────────────────────────────────────────────────
robot.wheel_radius  = 0.05;
robot.wheel_base    = 0.30;
robot.max_lin_vel   = 0.8;
robot.max_ang_vel   = 2.5;
robot.sensor_range  = 5.0;
robot.sensor_angles = [-pi/2, -pi/4, 0, pi/4, pi/2];

%% ─── Sensor Noise Parameters (same as Step 6) ───────────────────────────
sensor_params.odom_slip_factor = 0.02;
sensor_params.imu_gyro_noise   = 0.05;
sensor_params.imu_gyro_bias    = 0.008;
sensor_params.gps_pos_noise    = 0.8;
sensor_params.gps_update_rate  = 5;
sensor_params.lidar_range_noise = 0.03;

%% ─── EKF Parameters ─────────────────────────────────────────────────────
Q = diag([0.01^2, 0.01^2, 0.005^2]);
R_gps = diag([sensor_params.gps_pos_noise^2, sensor_params.gps_pos_noise^2]);
P_ekf = diag([0.1^2, 0.1^2, 0.05^2]);

fprintf('Sensor fusion: Odometry + IMU + GPS → EKF → Fuzzy FIS\n\n');

%% ─── Simulation Setup ────────────────────────────────────────────────────
dt    = 0.05;
t_max = 200;
t     = 0:dt:t_max;
N     = length(t);

% Initial heading toward goal
init_theta = atan2(goal_position(2) - start_position(2), ...
                   goal_position(1) - start_position(1));

% State arrays — track BOTH true and estimated positions
true_state  = zeros(N, 3);    % Ground truth (x, y, theta)
ekf_state   = zeros(N, 3);    % EKF estimate
true_state(1,:) = [start_position, init_theta];
ekf_state(1,:)  = true_state(1,:);

trajectory      = ekf_state;   % Robot drives using EKF estimate
velocity_log    = zeros(N, 2);
sensor_log      = zeros(N, 5); % 5 LiDAR beams
fuzzy_input_log = zeros(N, 3); % [dist_front, dist_left, heading_err]
P_log           = zeros(N, 3); % Covariance diagonal [σ²_x, σ²_y, σ²_θ]
P_log(1,:)      = diag(P_ekf)';
uncertainty_log = zeros(N, 1); % Scalar uncertainty used in fuzzy adaptation

% GPS state
imu_bias = sensor_params.imu_gyro_bias;

%% ─── Setup Figure ────────────────────────────────────────────────────────
figure('Name', 'Step 7: Fuzzy Navigation + EKF', 'Position', [30 30 1550 780]);

fprintf('Starting EKF-enhanced fuzzy navigation...\n');
fprintf('─────────────────────────────────────────\n');

goal_reached = false;
final_step   = N;

%% ═══════════════════════════════════════════════════════════════════════
%  MAIN LOOP
%% ═══════════════════════════════════════════════════════════════════════
for k = 1:N-1

    %% ── 1. Ground truth state ──────────────────────────────────────────
    x_true     = true_state(k, 1);
    y_true     = true_state(k, 2);
    theta_true = true_state(k, 3);

    % Current EKF estimate (what robot believes)
    x_ekf     = ekf_state(k, 1);
    y_ekf     = ekf_state(k, 2);
    theta_ekf = ekf_state(k, 3);

    %% ── 2. LiDAR — cast from TRUE position ────────────────────────────
    % LiDAR measures real distances but with range noise
    raw_dists = simulateLidar(x_true, y_true, theta_true, ...
                              robot.sensor_angles, robot.sensor_range, map);
    noisy_dists = max(0.1, raw_dists + sensor_params.lidar_range_noise * randn(size(raw_dists)));
    sensor_log(k,:) = noisy_dists;

    %% ── 3. Compute Fuzzy Inputs ───────────────────────────────────────
    dist_front = noisy_dists(3);              % Centre beam
    dist_left  = mean(noisy_dists(1:2));      % Left-side average

    % Heading error uses EKF estimate (not raw odometry)
    angle_to_goal = atan2(goal_position(2) - y_ekf, goal_position(1) - x_ekf);
    heading_err   = wrapToPi(angle_to_goal - theta_ekf);

    % ── Uncertainty-aware speed scaling ──────────────────────────────────
    % When EKF position uncertainty is high → be more conservative
    % sigma_pos = combined position uncertainty (metres)
    sigma_pos = sqrt(P_ekf(1,1) + P_ekf(2,2));
    
    % Scale factor: 1.0 = full speed, reduces to 0.5 when very uncertain
    % Students can tune these thresholds
    uncertainty_scale = max(0.5, 1.0 - sigma_pos * 0.8);
    uncertainty_log(k) = sigma_pos;

    % Clamp to FIS ranges
    dist_front  = max(0, min(5, dist_front));
    dist_left   = max(0, min(5, dist_left));
    heading_err = max(-pi, min(pi, heading_err));
    fuzzy_input_log(k,:) = [dist_front, dist_left, heading_err];

    %% ── 4. Evaluate Fuzzy FIS ─────────────────────────────────────────
    fuzzy_out = evalfis(fis, [dist_front, dist_left, heading_err]);
    v_fuzzy   = fuzzy_out(1);
    w_fuzzy   = fuzzy_out(2);

    % Apply uncertainty scaling to linear speed only
    % (angular velocity stays responsive for obstacle avoidance)
    v_cmd = v_fuzzy * uncertainty_scale;
    w_cmd = w_fuzzy;

    % Clamp to robot limits
    v_cmd = max(0, min(v_cmd, robot.max_lin_vel));
    w_cmd = max(-robot.max_ang_vel, min(w_cmd, robot.max_ang_vel));
    velocity_log(k,:) = [v_cmd, w_cmd];

    %% ── 5. True kinematic integration ────────────────────────────────
    x_true_new     = x_true + v_cmd * cos(theta_true) * dt;
    y_true_new     = y_true + v_cmd * sin(theta_true) * dt;
    theta_true_new = wrapToPi(theta_true + w_cmd * dt);

    x_true_new = max(0.3, min(map_width  - 0.3, x_true_new));
    y_true_new = max(0.3, min(map_height - 0.3, y_true_new));
    true_state(k+1,:) = [x_true_new, y_true_new, theta_true_new];

    %% ── 6. Odometry measurement (noisy wheel slip) ────────────────────
    slip_L = 1 + sensor_params.odom_slip_factor * randn();
    slip_R = 1 + sensor_params.odom_slip_factor * randn();
    v_odom = v_cmd * (slip_L + slip_R) / 2;
    w_odom = w_cmd * slip_R / slip_L;  % Asymmetric slip → heading drift

    %% ── 7. IMU measurement (gyro with bias + noise) ───────────────────
    imu_gyro = w_cmd + imu_bias + sensor_params.imu_gyro_noise * randn();

    %% ── 8. GPS measurement (sparse, noisy) ───────────────────────────
    gps_available = (mod(k, sensor_params.gps_update_rate) == 0);
    if gps_available
        gps_z = [x_true_new + sensor_params.gps_pos_noise * randn();
                 y_true_new + sensor_params.gps_pos_noise * randn()];
    end

    %% ── 9. EKF PREDICT ────────────────────────────────────────────────
    x_pred     = x_ekf + v_odom * cos(theta_ekf) * dt;
    y_pred     = y_ekf + v_odom * sin(theta_ekf) * dt;
    theta_pred = wrapToPi(theta_ekf + imu_gyro * dt);

    % Jacobian
    F = [1, 0, -v_odom * sin(theta_ekf) * dt;
         0, 1,  v_odom * cos(theta_ekf) * dt;
         0, 0,  1];

    P_ekf = F * P_ekf * F' + Q;

    %% ── 10. EKF UPDATE (GPS) ──────────────────────────────────────────
    if gps_available
        H         = [1, 0, 0; 0, 1, 0];
        innovation = gps_z - [x_pred; y_pred];
        S          = H * P_ekf * H' + R_gps;
        K          = P_ekf * H' / S;
        update     = K * innovation;
        x_pred     = x_pred     + update(1);
        y_pred     = y_pred     + update(2);
        theta_pred = wrapToPi(theta_pred + update(3));
        P_ekf      = (eye(3) - K * H) * P_ekf;
    end

    ekf_state(k+1,:) = [x_pred, y_pred, theta_pred];
    P_log(k+1,:)     = diag(P_ekf)';

    %% ── 11. Collision check (on TRUE position) ───────────────────────
    if checkOccupancy(map, [x_true_new, y_true_new])
        fprintf('  ✗ Collision at t=%.2f s\n', t(k));
        final_step = k;
        break;
    end

    %% ── 12. Goal check ────────────────────────────────────────────────
    dist_to_goal = norm([x_true_new, y_true_new] - goal_position);
    if dist_to_goal < 0.4
        fprintf('  ✓ GOAL REACHED at t = %.2f s!\n', t(k));
        goal_reached = true;
        final_step   = k + 1;
        break;
    end

    %% ── 13. Real-time visualisation (every 30 steps) ─────────────────
    if mod(k, 30) == 0 || k == 1
        time_vec = t(1:k);

        %% Panel 1: Map + trajectories
        subplot(2, 4, [1, 5]);
        cla; show(map); hold on;

        % True path (green)
        plot(true_state(1:k,1), true_state(1:k,2), 'g-', 'LineWidth', 2.5);
        % EKF estimate (blue)
        plot(ekf_state(1:k,1), ekf_state(1:k,2), 'b--', 'LineWidth', 1.8);

        % EKF uncertainty ellipse at current position
        drawEllipse(x_pred, y_pred, P_ekf(1,1), P_ekf(2,2), 'b');

        % LiDAR beams
        for bi = 1:5
            ba = theta_true + robot.sensor_angles(bi);
            bx_end = x_true + noisy_dists(bi) * cos(ba);
            by_end = y_true + noisy_dists(bi) * sin(ba);
            plot([x_true bx_end], [y_true by_end], 'y-', 'LineWidth', 0.8, 'Color', [1 0.7 0]);
        end

        % Robot body
        drawRobotSF(x_true, y_true, theta_true, robot.wheel_base);

        plot(start_position(1), start_position(2), 'go', 'MarkerSize', 14, ...
             'MarkerFaceColor', 'g', 'LineWidth', 2);
        plot(goal_position(1), goal_position(2), 'rp', 'MarkerSize', 18, ...
             'MarkerFaceColor', 'r', 'LineWidth', 2);

        title(sprintf('EKF-Fuzzy Navigation  t=%.1fs  σ_{pos}=%.2fm  dist=%.2fm', ...
              t(k), sqrt(P_ekf(1,1)+P_ekf(2,2)), dist_to_goal), 'FontSize', 11);
        legend({'True Path','EKF Estimate','Uncertainty','LiDAR','Robot','Start','Goal'}, ...
               'Location','northwest', 'FontSize', 7);

        %% Panel 2: Linear velocity
        subplot(2, 4, 2);
        plot(time_vec, velocity_log(1:k,1), 'b-', 'LineWidth', 1.5); hold off;
        xlabel('Time (s)'); ylabel('v (m/s)');
        title('Linear Velocity', 'FontSize', 10);
        grid on; ylim([0, 0.9]); xlim([0, max(t(k)+5,10)]);

        %% Panel 3: Angular velocity
        subplot(2, 4, 3);
        plot(time_vec, velocity_log(1:k,2), 'r-', 'LineWidth', 1.5); hold off;
        xlabel('Time (s)'); ylabel('ω (rad/s)');
        title('Angular Velocity', 'FontSize', 10);
        grid on; ylim([-3, 3]); xlim([0, max(t(k)+5,10)]);

        %% Panel 4: Fuzzy inputs over time
        subplot(2, 4, 4);
        plot(time_vec, fuzzy_input_log(1:k,1), 'b-', 'LineWidth', 1.2); hold on;
        plot(time_vec, fuzzy_input_log(1:k,2), 'r-', 'LineWidth', 1.2);
        plot(time_vec, rad2deg(fuzzy_input_log(1:k,3))/30, 'g--', 'LineWidth', 1.2);
        hold off;
        xlabel('Time (s)'); ylabel('Normalised');
        title('Fuzzy Inputs', 'FontSize', 10);
        legend('dist\_front (m)', 'dist\_left (m)', 'heading/30°', 'FontSize', 7);
        grid on; xlim([0, max(t(k)+5,10)]);

        %% Panel 5: EKF covariance
        subplot(2, 4, 6);
        plot(time_vec, sqrt(P_log(1:k,1))*100, 'b-', 'LineWidth', 1.5); hold on;
        plot(time_vec, sqrt(P_log(1:k,2))*100, 'r-', 'LineWidth', 1.5);
        hold off;
        xlabel('Time (s)'); ylabel('σ (cm)');
        title('EKF Position Uncertainty', 'FontSize', 10);
        legend('σ_x', 'σ_y', 'FontSize', 8);
        grid on; xlim([0, max(t(k)+5,10)]);

        %% Panel 6: Uncertainty-scaled speed
        subplot(2, 4, 7);
        plot(time_vec, uncertainty_log(1:k), 'm-', 'LineWidth', 1.5); hold on;
        yline(1.0/0.8, 'r--', 'Max slow-down', 'FontSize', 8);
        hold off;
        xlabel('Time (s)'); ylabel('σ_{pos} (m)');
        title('Position Uncertainty → Speed Scaling', 'FontSize', 10);
        grid on; xlim([0, max(t(k)+5,10)]);

        %% Panel 7: Heading comparison
        subplot(2, 4, 8);
        plot(time_vec, rad2deg(true_state(1:k,3)), 'g-', 'LineWidth', 2);   hold on;
        plot(time_vec, rad2deg(ekf_state(1:k,3)),  'b--', 'LineWidth', 1.5);
        hold off;
        xlabel('Time (s)'); ylabel('Heading (°)');
        title('True vs EKF Heading', 'FontSize', 10);
        legend('True θ', 'EKF θ̂', 'FontSize', 8);
        grid on; xlim([0, max(t(k)+5,10)]);

        sgtitle('STEP 7: Fuzzy Navigation + EKF Sensor Fusion', ...
                'FontSize', 13, 'FontWeight', 'bold');
        drawnow;
    end
end

%% ─── Trim data ───────────────────────────────────────────────────────────
fs = final_step;
true_state    = true_state(1:fs,:);
ekf_state     = ekf_state(1:fs,:);
velocity_log  = velocity_log(1:fs,:);
P_log         = P_log(1:fs,:);
uncertainty_log = uncertainty_log(1:fs);
time_data     = t(1:fs)';

%% ─── Final metrics ───────────────────────────────────────────────────────
total_distance  = sum(sqrt(diff(true_state(:,1)).^2 + diff(true_state(:,2)).^2));
mean_sigma      = mean(sqrt(P_log(:,1) + P_log(:,2))) * 100;
mean_speed      = mean(velocity_log(:,1));
final_error     = norm(true_state(end,1:2) - goal_position) * 100;

fprintf('─────────────────────────────────────────\n\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('FUZZY + EKF NAVIGATION RESULTS:\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  Goal Reached:              %s\n',   mat2str(goal_reached));
fprintf('  Total Time:                %.2f s\n', time_data(end));
fprintf('  Total Distance:            %.2f m\n', total_distance);
fprintf('  Mean Speed:                %.3f m/s\n', mean_speed);
fprintf('  Mean EKF σ_pos:            %.2f cm\n', mean_sigma);
fprintf('  Final Position Error:      %.2f cm\n', final_error);
fprintf('═══════════════════════════════════════════════════════════\n\n');

save('navigation_data.mat', 'true_state', 'ekf_state', 'velocity_log', ...
     'P_log', 'time_data', 'goal_reached', '-append');

fprintf('Data saved. Proceed to Step 8: Performance Analysis\n');
fprintf('═══════════════════════════════════════════════════════════\n');

%% ═══════════════════════════════════════════════════════════
%  HELPER FUNCTIONS
%% ═══════════════════════════════════════════════════════════

function dists = simulateLidar(x, y, theta, angles, max_range, map)
    dists = max_range * ones(1, length(angles));
    step  = 0.05;
    for a = 1:length(angles)
        beam_angle = theta + angles(a);
        for r = step:step:max_range
            rx = x + r * cos(beam_angle);
            ry = y + r * sin(beam_angle);
            if rx < 0 || rx >= map.XWorldLimits(2) || ...
               ry < 0 || ry >= map.YWorldLimits(2)
                dists(a) = r; break;
            end
            if checkOccupancy(map, [rx, ry])
                dists(a) = r; break;
            end
        end
    end
end

function drawEllipse(cx, cy, var_x, var_y, color)
    angles = linspace(0, 2*pi, 50);
    ex = cx + 3 * sqrt(var_x) * cos(angles);
    ey = cy + 3 * sqrt(var_y) * sin(angles);
    plot(ex, ey, '-', 'Color', color, 'LineWidth', 1.0);
end

function drawRobotSF(x, y, theta, L)
    r = L / 2;
    angs = linspace(0, 2*pi, 30);
    fill(x + r*cos(angs), y + r*sin(angs), [0.2 0.6 0.9], ...
         'FaceAlpha', 0.85, 'EdgeColor', 'k', 'LineWidth', 1.5);
    quiver(x, y, r*1.5*cos(theta), r*1.5*sin(theta), 0, ...
           'r', 'LineWidth', 2.5, 'MaxHeadSize', 1.5);
end

function fis = buildInlineFIS()
    % Minimal self-contained FIS matching step4_fuzzy_fis.m
    fis = mamfis('Name','RobotNavigation');
    fis = addInput(fis,[0 5],'Name','dist_front');
    fis = addMF(fis,'dist_front','trapmf',[0 0 0.3 0.6],'Name','Very_Close');
    fis = addMF(fis,'dist_front','trimf',[0.3 0.8 1.5],'Name','Close');
    fis = addMF(fis,'dist_front','trimf',[1.0 2.0 3.0],'Name','Medium');
    fis = addMF(fis,'dist_front','trapmf',[2.5 3.5 5.0 5.0],'Name','Far');
    fis = addInput(fis,[0 5],'Name','dist_left');
    fis = addMF(fis,'dist_left','trapmf',[0 0 0.3 0.7],'Name','Very_Close');
    fis = addMF(fis,'dist_left','trimf',[0.3 0.9 1.5],'Name','Close');
    fis = addMF(fis,'dist_left','trimf',[1.0 2.0 3.5],'Name','Medium');
    fis = addMF(fis,'dist_left','trapmf',[3.0 4.0 5.0 5.0],'Name','Far');
    fis = addInput(fis,[-pi pi],'Name','heading_err');
    fis = addMF(fis,'heading_err','trapmf',[-pi -pi -1.5 -0.5],'Name','Far_Left');
    fis = addMF(fis,'heading_err','trimf',[-1.2 -0.5 -0.1],'Name','Left');
    fis = addMF(fis,'heading_err','trimf',[-0.2 0.0 0.2],'Name','Straight');
    fis = addMF(fis,'heading_err','trimf',[0.1 0.5 1.2],'Name','Right');
    fis = addMF(fis,'heading_err','trapmf',[0.5 1.5 pi pi],'Name','Far_Right');
    fis = addOutput(fis,[0 0.8],'Name','linear_vel');
    fis = addMF(fis,'linear_vel','trimf',[0.0 0.0 0.15],'Name','Stop');
    fis = addMF(fis,'linear_vel','trimf',[0.0 0.2 0.4],'Name','Slow');
    fis = addMF(fis,'linear_vel','trimf',[0.2 0.45 0.65],'Name','Medium');
    fis = addMF(fis,'linear_vel','trimf',[0.5 0.8 0.8],'Name','Fast');
    fis = addOutput(fis,[-2.5 2.5],'Name','angular_vel');
    fis = addMF(fis,'angular_vel','trimf',[-2.5 -2.5 -1.2],'Name','Hard_Left');
    fis = addMF(fis,'angular_vel','trimf',[-2.0 -0.8 -0.2],'Name','Left');
    fis = addMF(fis,'angular_vel','trimf',[-0.3 0.0 0.3],'Name','Straight');
    fis = addMF(fis,'angular_vel','trimf',[0.2 0.8 2.0],'Name','Right');
    fis = addMF(fis,'angular_vel','trimf',[1.2 2.5 2.5],'Name','Hard_Right');
    ruleList = [
        1 0 0  1 1  1 1;
        2 1 0  1 1  1 1;
        2 2 0  2 4  1 1;
        2 3 0  2 4  1 1;
        2 4 0  2 3  1 1;
        3 1 0  2 1  1 1;
        3 3 3  3 3  1 1;
        3 3 1  3 1  1 1;
        3 3 5  3 5  1 1;
        4 4 3  4 3  1 1;
        4 4 1  4 1  1 1;
        4 4 5  4 5  1 1;
        4 1 0  2 1  1 1;
    ];
    fis = addRule(fis, ruleList);
end