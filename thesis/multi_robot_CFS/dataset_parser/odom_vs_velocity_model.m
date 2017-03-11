function [] = odom_vs_velocity_model(datamat)

% Load the saved robot specific dataset.
robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

% The length of the dataset
data_size = length(robot.odom);

% Loading all the odometry poses from the dataset
all_poses = [];
all_poses = [all_poses, robot.odom.pose];
pose_x = all_poses(1:3:end);
pose_y = all_poses(2:3:end);
pose_theta = all_poses(3:3:end);

% Getting all the poses based on the velocity model.
[model_x, model_y, model_theta] = test_ideal_model(datamat);

% Variable declaration for storing difference in distance between samples
% based on different models or raw data.
local_distance_difference = zeros(data_size-1, 1);
local_distance_model = zeros(data_size-1, 1);
local_distance_raw = zeros(data_size-1, 1);

% Variable declaration for storing the coordinate wise difference.
local_distance_raw_x = zeros(data_size-1, 1);
local_distance_raw_y = zeros(data_size-1, 1);
local_distance_raw_theta = zeros(data_size-1, 1);
local_distance_model_x = zeros(data_size-1, 1);
local_distance_model_y = zeros(data_size-1, 1);
local_distance_model_theta = zeros(data_size-1, 1);


for i=1:data_size-1
    local_distance_model(i) = distance_euclidean([model_x(i),model_y(i)], [model_x(i+1),model_y(i+1)]);
    local_distance_raw(i) = distance_euclidean([pose_x(i),pose_y(i)], [pose_x(i+1),pose_y(i+1)]);
    local_distance_difference(i) = local_distance_model(i) - local_distance_raw(i);
    
    local_distance_raw_x(i) = abs(pose_x(i) - pose_x(i+1));
    local_distance_raw_y(i) = abs(pose_y(i) - pose_y(i+1));
    local_distance_raw_theta(i) = wrapToPi(((pose_theta(i+1)) - (pose_theta(i))));
    local_distance_model_x(i) = abs(model_x(i) - model_x(i+1));
    local_distance_model_y(i) = abs(model_y(i) - model_y(i+1));
    local_distance_model_theta(i) = wrapToPi(wrapToPi(model_theta(i+1)) - wrapToPi(model_theta(i)));
end

% figure;
% plot(local_distance_difference);

figure;
hold on
% plot(local_distance_raw);
% plot(local_distance_model);

plot(cumsum(local_distance_raw))
plot(cumsum(local_distance_model))

% plot(local_distance_raw_theta);
% plot(local_distance_raw_x);
% plot(local_distance_model_x);

% 2D Euclidean distance
function dist = distance_euclidean(A, B)
dist = sqrt((A(1) - B(1))^2 + (A(2) - B(2))^2);

