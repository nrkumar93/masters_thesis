function [robot, opt_lin_vel, opt_ang_vel] = posterior_velocity_correction(datamat)

robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

meas_t = [];
meas_t = [meas_t, robot.odom.measurement_time];
del_t = diff(meas_t);

poses = [];
poses = [poses, robot.odom.pose];
pose_x = poses(1:3:end);
pose_y = poses(2:3:end);
pose_theta = poses(3:3:end);

opt_lin_vel = zeros(length(pose_x)-1, 1);
opt_ang_vel = zeros(length(pose_x)-1, 1);
gamma = zeros(length(pose_x)-1, 1);
del_theta = zeros(length(pose_x)-1, 1);

for i = 1:length(pose_x)-1
    mu = 0.5 * (((pose_x(i) - pose_x(i+1)) * cos(pose_theta(i)) + (pose_y(i) - pose_y(i+1)) * sin(pose_theta(i)))/ ...
                ((pose_y(i) - pose_y(i+1)) * cos(pose_theta(i)) - (pose_x(i) - pose_x(i+1)) * sin(pose_theta(i))));

    center_x = (pose_x(i) + pose_x(i+1))/2 + mu * (pose_y(i) - pose_y(i+1));
    center_y = (pose_y(i) + pose_y(i+1))/2 + mu * (pose_x(i+1) - pose_x(i));
    
    radius = sqrt((pose_x(i) - center_x)^2 + (pose_y(i) - center_y)^2);
    
    del_theta(i) = atan2(pose_y(i+1) - center_y, pose_x(i+1) - center_x) - ...
                   atan2(pose_y(i) - center_y, pose_x(i) - center_x);

    opt_lin_vel(i) = (del_theta(i)/del_t(i)) * radius;
    opt_ang_vel(i) = (del_theta(i)/del_t(i));
    
    gamma(i) = ((pose_theta(i+1) - pose_theta(i+1))/del_t(i)) - opt_ang_vel(i);

end

