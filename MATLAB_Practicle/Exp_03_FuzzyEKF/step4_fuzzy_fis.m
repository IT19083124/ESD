%% step4_fuzzy_fis.m
% LEARNING OBJECTIVE: Design a Mamdani FIS for obstacle avoidance navigation
%
% Students will learn:
% - How to define linguistic variables for robot sensing
% - How fuzzy rules encode human-like navigation logic
% - How to tune membership functions for robot behaviour

clc; clear; close all;

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   STEP 4: FUZZY LOGIC NAVIGATION SYSTEM DESIGN\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

%% ═══════════════════════════════════════════════════════════
%  FUZZY SYSTEM CONCEPT
%  Inputs  (what the robot "senses"):
%    1. dist_front  - distance to obstacle directly ahead (0–5 m)
%    2. dist_left   - distance to obstacle on left side  (0–5 m)
%    3. heading_err - angular error to goal              (-pi to pi rad)
%
%  Outputs (velocity commands):
%    1. linear_vel  - forward speed   (0 to 0.8 m/s)
%    2. angular_vel - turning rate   (-2.5 to 2.5 rad/s)
%% ═══════════════════════════════════════════════════════════

%% Create Mamdani FIS
fis = mamfis('Name', 'RobotNavigation');

fprintf('Creating Fuzzy Inference System...\n\n');

%% ─── INPUT 1: Distance to Front Obstacle ───────────────────
fis = addInput(fis, [0 5], 'Name', 'dist_front');

% Membership functions: Very_Close, Close, Medium, Far
fis = addMF(fis, 'dist_front', 'trapmf', [0 0 0.3 0.6],   'Name', 'Very_Close');
fis = addMF(fis, 'dist_front', 'trimf',  [0.3 0.8 1.5],   'Name', 'Close');
fis = addMF(fis, 'dist_front', 'trimf',  [1.0 2.0 3.0],   'Name', 'Medium');
fis = addMF(fis, 'dist_front', 'trapmf', [2.5 3.5 5.0 5.0],'Name', 'Far');

fprintf('Input 1: dist_front (0-5 m)\n');
fprintf('  MFs: Very_Close | Close | Medium | Far\n\n');

%% ─── INPUT 2: Distance to Left Obstacle ────────────────────
fis = addInput(fis, [0 5], 'Name', 'dist_left');

fis = addMF(fis, 'dist_left', 'trapmf', [0 0 0.3 0.7],    'Name', 'Very_Close');
fis = addMF(fis, 'dist_left', 'trimf',  [0.3 0.9 1.5],    'Name', 'Close');
fis = addMF(fis, 'dist_left', 'trimf',  [1.0 2.0 3.5],    'Name', 'Medium');
fis = addMF(fis, 'dist_left', 'trapmf', [3.0 4.0 5.0 5.0],'Name', 'Far');

fprintf('Input 2: dist_left (0-5 m)\n');
fprintf('  MFs: Very_Close | Close | Medium | Far\n\n');

%% ─── INPUT 3: Heading Error to Goal ────────────────────────
fis = addInput(fis, [-pi pi], 'Name', 'heading_err');

fis = addMF(fis, 'heading_err', 'trapmf', [-pi -pi -1.5 -0.5], 'Name', 'Far_Left');
fis = addMF(fis, 'heading_err', 'trimf',  [-1.2 -0.5 -0.1],    'Name', 'Left');
fis = addMF(fis, 'heading_err', 'trimf',  [-0.2 0.0 0.2],      'Name', 'Straight');
fis = addMF(fis, 'heading_err', 'trimf',  [0.1 0.5 1.2],       'Name', 'Right');
fis = addMF(fis, 'heading_err', 'trapmf', [0.5 1.5 pi pi],     'Name', 'Far_Right');

fprintf('Input 3: heading_err (-π to π rad)\n');
fprintf('  MFs: Far_Left | Left | Straight | Right | Far_Right\n\n');

%% ─── OUTPUT 1: Linear Velocity ──────────────────────────────
fis = addOutput(fis, [0 0.8], 'Name', 'linear_vel');

fis = addMF(fis, 'linear_vel', 'trimf', [0.0 0.0 0.15],  'Name', 'Stop');
fis = addMF(fis, 'linear_vel', 'trimf', [0.0 0.2 0.4],   'Name', 'Slow');
fis = addMF(fis, 'linear_vel', 'trimf', [0.2 0.45 0.65], 'Name', 'Medium');
fis = addMF(fis, 'linear_vel', 'trimf', [0.5 0.8 0.8],   'Name', 'Fast');

fprintf('Output 1: linear_vel (0-0.8 m/s)\n');
fprintf('  MFs: Stop | Slow | Medium | Fast\n\n');

%% ─── OUTPUT 2: Angular Velocity ─────────────────────────────
fis = addOutput(fis, [-2.5 2.5], 'Name', 'angular_vel');

fis = addMF(fis, 'angular_vel', 'trimf', [-2.5 -2.5 -1.2], 'Name', 'Hard_Left');
fis = addMF(fis, 'angular_vel', 'trimf', [-2.0 -0.8 -0.2], 'Name', 'Left');
fis = addMF(fis, 'angular_vel', 'trimf', [-0.3  0.0  0.3], 'Name', 'Straight');
fis = addMF(fis, 'angular_vel', 'trimf', [ 0.2  0.8  2.0], 'Name', 'Right');
fis = addMF(fis, 'angular_vel', 'trimf', [ 1.2  2.5  2.5], 'Name', 'Hard_Right');

fprintf('Output 2: angular_vel (-2.5 to 2.5 rad/s)\n');
fprintf('  MFs: Hard_Left | Left | Straight | Right | Hard_Right\n\n');

%% ═══════════════════════════════════════════════════════════
%  FUZZY RULE BASE
%  Format: [in1 in2 in3, out1 out2, weight, operator]
%    in values: 1=Very_Close/Far_Left, 2=Close/Left, 3=Medium/Straight,
%               4=Far/Right, 0=don't care
%    operator: 1=AND, 2=OR
%% ═══════════════════════════════════════════════════════════

fprintf('Defining fuzzy rule base...\n');

ruleList = [
    % dist_front  dist_left  heading_err  | lin_vel  ang_vel  | weight  op
    % ─── EMERGENCY: obstacle very close in front ───────────────────────
    1  0  0    1  1  1  1;   % Very_Close front, any left, any heading → Stop, Hard_Left
    1  0  0    1  2  1  1;   % Very_Close front → Stop (alternative turn)

    % ─── OBSTACLE CLOSE IN FRONT: slow and turn ────────────────────────
    2  1  0    1  1  1  1;   % Close front + Very_Close left → Stop, Hard_Left
    2  2  0    2  4  1  1;   % Close front + Close left → Slow, Right turn
    2  3  0    2  4  1  1;   % Close front + Medium left → Slow, Right
    2  4  0    2  3  1  1;   % Close front + Far left → Slow, Straight
    2  0  0    2  4  1  1;   % Close front, any left → Slow, turn Right

    % ─── MEDIUM DISTANCE: moderate navigation ──────────────────────────
    3  1  0    2  1  1  1;   % Medium front + Very_Close left → Slow, Hard_Left
    3  2  0    2  3  1  1;   % Medium front + Close left → Slow, Straight
    3  3  1    3  1  1  1;   % Medium front + Medium left + heading Far_Left → Medium, Hard_Left
    3  3  2    3  2  1  1;   % Medium front + Medium left + heading Left → Medium, Left
    3  3  3    3  3  1  1;   % Medium front + Medium left + heading Straight → Medium, Straight
    3  3  4    3  4  1  1;   % Medium front + Medium left + heading Right → Medium, Right
    3  3  5    3  5  1  1;   % Medium front + Medium left + heading Far_Right → Medium, Hard_Right
    3  4  3    3  3  1  1;   % Medium front + Far left + Straight → Medium, Straight

    % ─── CLEAR PATH: fast, goal-directed navigation ────────────────────
    4  4  1    4  1  1  1;   % Far front + Far left + Far_Left heading → Fast, Hard_Left
    4  4  2    4  2  1  1;   % Far front + Far left + Left → Fast, Left
    4  4  3    4  3  1  1;   % Far front + Far left + Straight → Fast, Straight
    4  4  4    4  4  1  1;   % Far front + Far left + Right → Fast, Right
    4  4  5    4  5  1  1;   % Far front + Far left + Far_Right → Fast, Hard_Right
    4  3  3    3  3  1  1;   % Far front + Medium left + Straight → Medium, Straight
    4  2  3    2  3  1  1;   % Far front + Close left → Slow, Straight
    4  1  0    2  1  1  1;   % Far front + Very_Close left → Slow, Hard_Left (wall hug avoidance)
];

fis = addRule(fis, ruleList);

fprintf('  Added %d rules to the rule base.\n\n', size(ruleList, 1));

%% Verify FIS
fprintf('Verifying FIS with sample inputs...\n');
test_inputs = [
    1.0, 2.0, 0.0;    % Close front, medium left, straight heading
    3.0, 3.0, 0.5;    % Medium distances, slight right heading
    0.2, 0.5, 1.0;    % Very close front, emergency
    4.5, 4.5, 0.0;    % Clear path, straight ahead
];

fprintf('  %-15s %-12s %-14s | %-12s %-14s\n', ...
        'dist_front', 'dist_left', 'heading_err', 'linear_vel', 'angular_vel');
fprintf('  %s\n', repmat('-', 1, 65));

for i = 1:size(test_inputs, 1)
    result = evalfis(fis, test_inputs(i,:));
    fprintf('  %-15.1f %-12.1f %-14.2f | %-12.3f %-14.3f\n', ...
            test_inputs(i,1), test_inputs(i,2), test_inputs(i,3), ...
            result(1), result(2));
end

%% Visualize Membership Functions
figure('Name', 'Step 4: Fuzzy System Design', 'Position', [50 50 1400 700]);

plotTitles = {'Input 1: Distance to Front', 'Input 2: Distance to Left', ...
              'Input 3: Heading Error', 'Output 1: Linear Velocity', ...
              'Output 2: Angular Velocity'};

for i = 1:5
    subplot(2, 3, i);
    if i <= 3
        plotmf(fis, 'input', i);
    else
        plotmf(fis, 'output', i-3);
    end
    title(plotTitles{i}, 'FontSize', 11);
    grid on;
end

% Rule surface (dist_front vs heading_err → linear_vel)
subplot(2, 3, 6);
gensurf(fis, [1 3], 1);
xlabel('dist\_front'); ylabel('heading\_err'); zlabel('linear\_vel');
title('Rule Surface: Speed Control', 'FontSize', 11);

sgtitle('STEP 4: Fuzzy Navigation System', 'FontSize', 14, 'FontWeight', 'bold');

%% Save FIS
writeFIS(fis, 'robot_navigation_fis');
save('navigation_data.mat', 'fis', '-append');

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('FIS saved: robot_navigation_fis.fis\n');
fprintf('Proceed to Step 5: Fuzzy Navigation Simulation\n');
fprintf('═══════════════════════════════════════════════════════════\n');