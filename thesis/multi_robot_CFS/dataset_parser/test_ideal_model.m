function [ideal_x, ideal_y, ideal_theta] = test_ideal_model(datamat)

robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

plot_offset = 4000;

% The length of the dataset
data_size = length(robot.odom);

all_poses = [];
all_poses = [all_poses, robot.odom.pose];
pose_x = all_poses(1:3:end);
pose_y = all_poses(2:3:end);
pose_theta = all_poses(3:3:end);

[ideal_x, ideal_y, ideal_theta] = ideal_model(datamat);

figure;
hold on
for j=plot_offset:data_size
    plot(pose_x(j), pose_y(j), 'b*');
    plot(ideal_x(j), ideal_y(j), 'r*');
    pause(0.01)
    j
end
