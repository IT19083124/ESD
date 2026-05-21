%% step6_sensor_fusion.m
% LEARNING OBJECTIVE: Sensor Fusion using Extended Kalman Filter (EKF)

% SENSORS SIMULATED:
%   Sensor 1 — LiDAR (5-beam ray-cast, existing from fuzzy navigator)
%   Sensor 2 — Wheel Odometry (integrates encoder ticks, drifts over time)
%   Sensor 3 — IMU (gyroscope heading rate, with bias + noise)
%   Sensor 4 — GPS/GNSS (sparse, noisy absolute position fix)
%
% EKF FUSES: Odometry (predict) + IMU (predict) + GPS (update when available)

clc; clear; close all;

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   STEP 6: SENSOR FUSION WITH EXTENDED KALMAN FILTER\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% ─── Load Environment ────────────────────────────────────────────────────
if ~exist('navigation_data.mat', 'file')
    error('Run Step 3 first to create navigation_data.mat!');
end
load('navigation_data.mat');
fprintf('Environment loaded: %d x %d m map\n\n', map_width, map_height);

%% ─── Robot Physical Parameters ───────────────────────────────────────────
robot.wheel_radius  = 0.05;    % m
robot.wheel_base    = 0.30;    % m
robot.max_lin_vel   = 0.8;     % m/s
robot.max_ang_vel   = 2.5;     % rad/s
robot.encoder_ticks = 1000;    % ticks per revolution

%% ─── Sensor Noise Parameters (realistic values) ─────────────────────────
% These represent real hardware specifications.
% Higher sigma = noisier sensor.

sensor_params.lidar_range_noise   = 0.03;   % LiDAR range std dev (m)
sensor_params.lidar_angle_noise   = 0.01;   % LiDAR angle std dev (rad)
sensor_params.odom_slip_factor    = 0.02;   % Wheel slip (2% error per step)
sensor_params.imu_gyro_noise      = 0.05;   % Gyro noise std dev (rad/s)
sensor_params.imu_gyro_bias       = 0.008;  % Gyro constant bias (rad/s)
sensor_params.gps_pos_noise       = 0.8;    % GPS position std dev (m)
sensor_params.gps_update_rate     = 5;      % GPS update every N steps (Hz sim)

fprintf('Sensor Configuration:\n');
fprintf('  LiDAR:      range noise ±%.0f cm\n', sensor_params.lidar_range_noise*100);
fprintf('  Odometry:   slip factor %.1f%%\n', sensor_params.odom_slip_factor*100);
fprintf('  IMU Gyro:   noise ±%.0f mrad/s, bias %.0f mrad/s\n', ...
        sensor_params.imu_gyro_noise*1000, sensor_params.imu_gyro_bias*1000);
fprintf('  GPS:        position noise ±%.1f m, rate 1/%d steps\n\n', ...
        sensor_params.gps_pos_noise, sensor_params.gps_update_rate);

%% ─── EKF Parameters ──────────────────────────────────────────────────────
% State vector: [x, y, theta]  (3x1)
%
% Process noise Q: uncertainty injected by motion model each step
% Measurement noise R_gps: GPS measurement noise covariance
%
% Tuning rule of thumb:
%   Q controls how much the filter trusts its own motion model
%   R controls how much the filter trusts sensor measurements
%   Large Q → filter reacts quickly to measurements (sensor-heavy)
%   Small Q → filter relies more on model (model-heavy)

ekf.Q = diag([0.01^2, 0.01^2, 0.005^2]);    % Process noise covariance
ekf.R_gps = diag([sensor_params.gps_pos_noise^2, ...
                   sensor_params.gps_pos_noise^2]);  % GPS measurement noise

% Initial state uncertainty (we know start position well)
ekf.P = diag([0.1^2, 0.1^2, 0.05^2]);       % Initial covariance

fprintf('EKF Parameters:\n');
fprintf('  Process noise Q  (pos): ±%.0f mm, (heading): ±%.0f mrad\n', ...
        sqrt(ekf.Q(1,1))*1000, sqrt(ekf.Q(3,3))*1000);
fprintf('  GPS noise R      (pos): ±%.0f cm\n', sqrt(ekf.R_gps(1,1))*100);
fprintf('  Initial P        (pos): ±%.0f cm\n\n', sqrt(ekf.P(1,1))*100);

%% ─── Simulation Setup ────────────────────────────────────────────────────
dt    = 0.05;
t_max = 120;
t     = 0:dt:t_max;
N     = length(t);

% True robot state (ground truth, what "really" happens)
dx_init = goal_position(1) - start_position(1);
dy_init = goal_position(2) - start_position(2);
true_state    = zeros(N, 3);
true_state(1,:) = [start_position, atan2(dy_init, dx_init)];

% EKF estimated state
ekf_state     = zeros(N, 3);
ekf_state(1,:) = true_state(1,:);

% Raw sensor readings (before fusion)
odom_state    = zeros(N, 3);   % Odometry-only dead reckoning
odom_state(1,:) = true_state(1,:);

% Storage for analysis
P_history     = zeros(N, 3);   % Diagonal of covariance (uncertainty)
P_history(1,:) = diag(ekf.P)';

gps_measurements = NaN(N, 2);  % GPS fixes (sparse)
imu_readings     = zeros(N, 1); % IMU gyro readings

% IMU bias state (slowly drifting)
imu_bias = sensor_params.imu_gyro_bias;

% EKF covariance (evolves over time)
P_ekf = ekf.P;

sensor_labels = zeros(N, 1);   % 0=predict, 1=GPS update

fprintf('Starting sensor fusion simulation...\n');
fprintf('─────────────────────────────────────────\n');

%% ─── Velocity command generator (simple goal-seeking for demo) ───────────
% Use a simple proportional controller to drive toward goal
% (In step7 the fuzzy FIS replaces this)
function [v, omega] = simpleGoalController(x, y, theta, gx, gy, max_v, max_w)
    dist        = norm([gx-x, gy-y]);
    angle_goal  = atan2(gy-y, gx-x);
    heading_err = atan2(sin(angle_goal-theta), cos(angle_goal-theta));
    v     = min(0.6, dist * 0.4);
    omega = min(max_w, max(-max_w, 2.5 * heading_err));
end

%% ═══════════════════════════════════════════════════════════
%  MAIN SIMULATION LOOP
%  Each iteration:
%   1. Generate ground-truth motion (what robot truly does)
%   2. Simulate each sensor with noise
%   3. EKF Predict step (motion model)
%   4. EKF Update step (GPS when available)
%   5. Log everything
%% ═══════════════════════════════════════════════════════════

for k = 1:N-1
    %% ─ 1. TRUE ROBOT MOTION ─────────────────────────────────────────────
    x_true     = true_state(k, 1);
    y_true     = true_state(k, 2);
    theta_true = true_state(k, 3);

    [v_cmd, w_cmd] = simpleGoalController(x_true, y_true, theta_true, ...
        goal_position(1), goal_position(2), robot.max_lin_vel, robot.max_ang_vel);

    % True kinematics (perfect integration)
    x_true_new     = x_true + v_cmd * cos(theta_true) * dt;
    y_true_new     = y_true + v_cmd * sin(theta_true) * dt;
    theta_true_new = wrapToPi(theta_true + w_cmd * dt);
    true_state(k+1,:) = [x_true_new, y_true_new, theta_true_new];

    %% ─ 2. SENSOR SIMULATION ─────────────────────────────────────────────

    % ── SENSOR A: Wheel Odometry ─────────────────────────────────────────
    % Model: each wheel has random slip noise proportional to speed
    % This causes heading drift and position error that grows over time.
    slip_L = 1 + sensor_params.odom_slip_factor * randn();
    slip_R = 1 + sensor_params.odom_slip_factor * randn();
    v_L_noisy = v_cmd * slip_L;    % Left wheel noisy speed
    v_R_noisy = v_cmd * slip_R;    % Right wheel noisy speed

    v_odom   = (v_L_noisy + v_R_noisy) / 2;
    w_odom   = (v_R_noisy - v_L_noisy) / robot.wheel_base;

    % Dead-reckoning using noisy odometry
    x_odom     = odom_state(k, 1);
    y_odom     = odom_state(k, 2);
    theta_odom = odom_state(k, 3);

    x_odom_new     = x_odom + v_odom * cos(theta_odom) * dt;
    y_odom_new     = y_odom + v_odom * sin(theta_odom) * dt;
    theta_odom_new = wrapToPi(theta_odom + w_odom * dt);
    odom_state(k+1,:) = [x_odom_new, y_odom_new, theta_odom_new];

    % ── SENSOR B: IMU Gyroscope ───────────────────────────────────────────
    % Model: true angular rate + constant bias + white noise
    imu_gyro = w_cmd + imu_bias + sensor_params.imu_gyro_noise * randn();
    imu_readings(k) = imu_gyro;

    % ── SENSOR C: GPS ─────────────────────────────────────────────────────
    % Model: sparse (updates every N steps) with Gaussian position noise
    gps_available = (mod(k, sensor_params.gps_update_rate) == 0);
    if gps_available
        gps_x = x_true_new + sensor_params.gps_pos_noise * randn();
        gps_y = y_true_new + sensor_params.gps_pos_noise * randn();
        gps_measurements(k+1,:) = [gps_x, gps_y];
        sensor_labels(k+1) = 1;
    end

    %% ─ 3. EKF PREDICT STEP ──────────────────────────────────────────────
    % Use odometry as the motion model (noisy but continuous)
    % State prediction: x_k|k-1 = f(x_k-1, u_k)
    x_ekf     = ekf_state(k, 1);
    y_ekf     = ekf_state(k, 2);
    theta_ekf = ekf_state(k, 3);

    % Predicted state using odometry velocities
    x_pred     = x_ekf + v_odom * cos(theta_ekf) * dt;
    y_pred     = y_ekf + v_odom * sin(theta_ekf) * dt;
    theta_pred = wrapToPi(theta_ekf + imu_gyro * dt);  % IMU for heading

    % Jacobian of motion model F_k = df/dx at current state
    %   df1/dtheta = -v*sin(theta)*dt
    %   df2/dtheta =  v*cos(theta)*dt
    F = [1, 0, -v_odom * sin(theta_ekf) * dt;
         0, 1,  v_odom * cos(theta_ekf) * dt;
         0, 0,  1];

    % Predicted covariance: P_k|k-1 = F * P_k-1 * F' + Q
    P_ekf = F * P_ekf * F' + ekf.Q;

    %% ─ 4. EKF UPDATE STEP (GPS) ─────────────────────────────────────────
    if gps_available
        % Measurement model: h(x) = [x, y]  (GPS measures position directly)
        z_gps = gps_measurements(k+1,:)';          % GPS measurement vector
        h_pred = [x_pred; y_pred];                  % Predicted measurement

        % Measurement Jacobian: H = dh/dx (identity for position states)
        H = [1, 0, 0;
             0, 1, 0];

        % Innovation: difference between measurement and prediction
        innovation = z_gps - h_pred;

        % Innovation covariance: S = H * P * H' + R
        S = H * P_ekf * H' + ekf.R_gps;

        % Kalman gain: K = P * H' * S^-1
        % Large K → trust GPS more; small K → trust prediction more
        K = P_ekf * H' / S;

        % State update: x_k = x_k|k-1 + K * innovation
        update_vec = K * innovation;
        x_pred     = x_pred     + update_vec(1);
        y_pred     = y_pred     + update_vec(2);
        theta_pred = wrapToPi(theta_pred + update_vec(3));

        % Covariance update (Joseph form — numerically stable):
        % P_k = (I - K*H) * P_k|k-1
        I_KH   = eye(3) - K * H;
        P_ekf  = I_KH * P_ekf;
    end

    % Store EKF output
    ekf_state(k+1,:) = [x_pred, y_pred, theta_pred];
    P_history(k+1,:) = diag(P_ekf)';

    % Goal check
    if norm([x_pred, y_pred] - goal_position) < 0.4
        fprintf('  ✓ EKF estimate reached goal at t = %.2f s!\n', t(k));
        true_state  = true_state(1:k+1,:);
        ekf_state   = ekf_state(1:k+1,:);
        odom_state  = odom_state(1:k+1,:);
        P_history   = P_history(1:k+1,:);
        imu_readings = imu_readings(1:k+1);
        gps_measurements = gps_measurements(1:k+1,:);
        sensor_labels = sensor_labels(1:k+1);
        t = t(1:k+1);
        N_final = k+1;
        break;
    end
    N_final = k+1;
end

fprintf('Simulation complete. %d timesteps.\n\n', N_final);

%% ─── Position Error Analysis ─────────────────────────────────────────────
true_pos  = true_state(1:N_final, 1:2);
ekf_pos   = ekf_state(1:N_final, 1:2);
odom_pos  = odom_state(1:N_final, 1:2);

ekf_pos_err  = sqrt(sum((ekf_pos  - true_pos).^2, 2)) * 100;  % cm
odom_pos_err = sqrt(sum((odom_pos - true_pos).^2, 2)) * 100;  % cm

%% ─── VISUALISATION ───────────────────────────────────────────────────────
figure('Name', 'Step 6: Sensor Fusion — EKF', 'Position', [50 50 1500 750]);

%% Panel 1: Trajectory comparison
subplot(2, 3, [1, 4]);
show(map); hold on;

plot(true_state(1:N_final,1), true_state(1:N_final,2), 'g-', 'LineWidth', 3);
plot(odom_state(1:N_final,1), odom_state(1:N_final,2), 'r--', 'LineWidth', 1.8);
plot(ekf_state(1:N_final,1),  ekf_state(1:N_final,2),  'b-',  'LineWidth', 2.5);

% GPS measurement dots
gps_valid = ~isnan(gps_measurements(1:N_final,1));
if any(gps_valid)
    plot(gps_measurements(gps_valid,1), gps_measurements(gps_valid,2), ...
         'k+', 'MarkerSize', 8, 'LineWidth', 1.5);
end

% Draw EKF uncertainty ellipses at intervals
draw_interval = max(1, floor(N_final/8));
for k = 1:draw_interval:N_final
    drawUncertaintyEllipse(ekf_state(k,1), ekf_state(k,2), ...
                           P_history(k,1), P_history(k,2), 'b');
end

plot(start_position(1), start_position(2), 'go', 'MarkerSize', 14, ...
     'MarkerFaceColor', 'g', 'LineWidth', 2);
plot(goal_position(1), goal_position(2), 'rp', 'MarkerSize', 18, ...
     'MarkerFaceColor', 'r', 'LineWidth', 2);

title('Trajectory Comparison', 'FontSize', 13);
legend({'True Path', 'Odometry Only (drifts)', 'EKF Fused Estimate', ...
        'GPS Fixes', 'Uncertainty Ellipse', 'Start', 'Goal'}, ...
       'Location', 'northwest', 'FontSize', 8);

%% Panel 2: Position error over time
subplot(2, 3, 2);
time_vec = t(1:N_final);
plot(time_vec, odom_pos_err, 'r-', 'LineWidth', 1.5); hold on;
plot(time_vec, ekf_pos_err,  'b-', 'LineWidth', 2.0);
% Mark GPS updates
gps_times = time_vec(sensor_labels(1:N_final)==1);
if ~isempty(gps_times)
    xline(gps_times, 'k:', 'Alpha', 0.4);
end
hold off;
xlabel('Time (s)'); ylabel('Position Error (cm)');
title('Position Error: Odometry vs EKF', 'FontSize', 11);
legend('Odometry alone', 'EKF fused', 'GPS updates', 'FontSize', 9);
grid on;

%% Panel 3: EKF covariance (uncertainty) evolution
subplot(2, 3, 3);
plot(time_vec, sqrt(P_history(1:N_final,1))*100, 'b-',  'LineWidth', 1.5); hold on;
plot(time_vec, sqrt(P_history(1:N_final,2))*100, 'r-',  'LineWidth', 1.5);
plot(time_vec, sqrt(P_history(1:N_final,3))*1000,'g--', 'LineWidth', 1.5);
if ~isempty(gps_times)
    xline(gps_times, 'k:', 'Alpha', 0.4);
end
hold off;
xlabel('Time (s)'); ylabel('1σ Uncertainty');
title('EKF Covariance (Uncertainty)', 'FontSize', 11);
legend('σ_x (cm)', 'σ_y (cm)', 'σ_θ (mrad)', 'GPS', 'FontSize', 9);
grid on;

%% Panel 4: IMU gyro readings vs true angular velocity
subplot(2, 3, 5);
% Reconstruct true omega from state differences
true_omega = diff(true_state(1:N_final,3)) ./ diff(t(1:N_final))';
true_omega(abs(true_omega) > 10) = NaN;  % Remove wrap-around artifacts

plot(time_vec(1:end-1), true_omega, 'g-', 'LineWidth', 1.5); hold on;
plot(time_vec(1:end-1), imu_readings(1:N_final-1), 'r-', 'LineWidth', 1.0);
hold off;
xlabel('Time (s)'); ylabel('Angular Rate (rad/s)');
title(sprintf('IMU Gyro vs True ω\n(bias=%.0f mrad/s)', imu_bias*1000), 'FontSize', 11);
legend('True ω', 'IMU (noisy+bias)', 'FontSize', 9);
grid on;

%% Panel 5: Heading comparison
subplot(2, 3, 6);
plot(time_vec, rad2deg(true_state(1:N_final,3)),  'g-', 'LineWidth', 2);   hold on;
plot(time_vec, rad2deg(odom_state(1:N_final,3)),  'r--', 'LineWidth', 1.5);
plot(time_vec, rad2deg(ekf_state(1:N_final,3)),   'b-',  'LineWidth', 2);
hold off;
xlabel('Time (s)'); ylabel('Heading (deg)');
title('Heading Estimate Comparison', 'FontSize', 11);
legend('True θ', 'Odometry θ', 'EKF θ', 'FontSize', 9);
grid on;

sgtitle('STEP 6: Sensor Fusion — Extended Kalman Filter', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ─── Print Summary Statistics ────────────────────────────────────────────
mean_odom_err = mean(odom_pos_err);
mean_ekf_err  = mean(ekf_pos_err);
max_odom_err  = max(odom_pos_err);
max_ekf_err   = max(ekf_pos_err);
improvement   = (mean_odom_err - mean_ekf_err) / mean_odom_err * 100;

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('SENSOR FUSION RESULTS:\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('%-25s %-15s %-15s\n', 'Metric', 'Odometry Only', 'EKF Fused');
fprintf('%s\n', repmat('-', 1, 55));
fprintf('%-25s %-15.2f %-15.2f\n', 'Mean pos. error (cm)', mean_odom_err, mean_ekf_err);
fprintf('%-25s %-15.2f %-15.2f\n', 'Max pos. error (cm)',  max_odom_err,  max_ekf_err);
fprintf('%s\n', repmat('-', 1, 55));
fprintf('EKF improvement over odometry: %.1f%%\n', improvement);
fprintf('GPS update count: %d fixes in %.0f s\n', sum(sensor_labels==1), t(end));
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% ─── Save fusion data for Step 7 ────────────────────────────────────────
sensor_fusion.ekf_state     = ekf_state(1:N_final,:);
sensor_fusion.true_state    = true_state(1:N_final,:);
sensor_fusion.odom_state    = odom_state(1:N_final,:);
sensor_fusion.P_history     = P_history(1:N_final,:);
sensor_fusion.sensor_params = sensor_params;
sensor_fusion.ekf_params    = ekf;
sensor_fusion.N_final       = N_final;
sensor_fusion.time_vec      = t(1:N_final);

save('navigation_data.mat', 'sensor_fusion', '-append');

fprintf('Sensor fusion data saved to navigation_data.mat\n');
fprintf('Proceed to Step 7: Fuzzy Navigator with EKF\n');
fprintf('═══════════════════════════════════════════════════════════\n');

%% ═══════════════════════════════════════════════════════════
%  HELPER FUNCTIONS
%% ═══════════════════════════════════════════════════════════

function drawUncertaintyEllipse(cx, cy, var_x, var_y, color)
    % Draw 1-sigma uncertainty ellipse at (cx, cy)
    angles = linspace(0, 2*pi, 40);
    ex = cx + 3 * sqrt(var_x) * cos(angles);  % 3-sigma
    ey = cy + 3 * sqrt(var_y) * sin(angles);
    plot(ex, ey, '-', 'Color', color, 'LineWidth', 0.8, 'MarkerSize', 1);
end