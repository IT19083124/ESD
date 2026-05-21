%% step1_setup_project.m
% Creates the project folder structure

clc; clear; close all;

% Create main project folder
project_name = 'RobotNavigation';
if ~exist(project_name, 'dir')
    mkdir(project_name);
end

% Create subfolders
subfolders = {'scripts', 'models', 'data', 'results', 'functions'};
for i = 1:length(subfolders)
    folder_path = fullfile(project_name, subfolders{i});
    if ~exist(folder_path, 'dir')
        mkdir(folder_path);
    end
end

% Add to path
addpath(genpath(project_name));

fprintf('Project structure created:\n');
fprintf('%s/\n', project_name);
for i = 1:length(subfolders)
    fprintf('  ├── %s/\n', subfolders{i});
end

% Save initial parameters
fprintf('\nProject setup complete!\n');
fprintf('Current directory: %s\n', pwd);