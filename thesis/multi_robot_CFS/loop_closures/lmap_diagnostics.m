clear;

%% Load the saved robot specific dataset.
datamat = './data/dataset_robot3.mat';
robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

data_start_point = 4000;
data_end_point = length(robot.odom);

%% Examining with a part of the entire dataset.
robot = data_chopper(robot, data_start_point, data_end_point);

%% The length of the dataset
data_size = length(robot.odom);
start_offset = 0;
end_offset = 0;

%% Loading lmap from the data.
lmap_poses = [];
lmap_poses = [lmap_poses, robot.lmap.pose];
lmap_x = lmap_poses(1:3:end);
lmap_y = lmap_poses(2:3:end);
lmap_theta = lmap_poses(3:3:end);

vec_u = 0.001 * cos(lmap_theta);
vec_v = 0.001 * sin(lmap_theta);

%% Load the loop closure points here.
X = [];
Y = [];
% % Robot 3. DO NOT REPLACE. ENTER NEW FOR  A DIFFERENT ROBOT.
% X = [-6.824 -7.45 -10.24 -9.52 -9.692 4.308 6.012 -0.883 -5.49 -9.135];
% Y = [ 6.207  7.34  15.37  11.56 12.04 14.62 14.93  14.14  16.69 9.892];


figure; 
hold on;
plot3(lmap_x, lmap_y, 1:length(lmap_x));
% quiver(lmap_x(1:15:end), lmap_y(1:15:end), vec_u(1:15:end), vec_v(1:15:end));
plot_closure_lines(X, Y, data_size);

function [] = plot_closure_lines(X, Y, data_size)
    assert(length(X) == length(Y));
    for i=1:length(X)
        plot3([X(i) X(i)], [Y(i) Y(i)], [0 data_size/2], 'k');
    end
end