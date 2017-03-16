function [] = plot_raw_odom(datamat)

% Load the saved robot specific dataset.
robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

all_poses = [];
all_poses = [all_poses, robot.odom.pose];

pose_x = all_poses(1:3:end);
pose_y = all_poses(2:3:end);
pose_theta = all_poses(3:3:end);

figure;
plot(pose_x, pose_y);
