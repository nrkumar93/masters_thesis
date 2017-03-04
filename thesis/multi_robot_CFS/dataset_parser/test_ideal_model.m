gamma = 0.0;

robot = load('dataset_robot3.mat');
robot = robot.dataset_robot3;

meas_t = [];
meas_t = [meas_t, robot.odom.measurement_time];
del_t = diff(meas_t);
% del_t = del_t(1:5000);

poses = [];
poses = [poses, robot.odom.pose];
vels = [];
vels = [vels, robot.odom.velocity];

net_vel = vels(1:3:end);
ang_vel = vels(3:3:end);

ideal_x = zeros(length(del_t) + 1, 1);
ideal_y = zeros(length(del_t) + 1, 1);
ideal_theta = zeros(length(del_t) + 1, 1);

for i = 1:length(del_t)
    if ang_vel(i) ~= 0
        ideal_x(i+1) = ideal_x(i) - ((net_vel(i)/ang_vel(i)) * sin(ideal_theta(i))) + ((net_vel(i)/ang_vel(i)) * sin(ideal_theta(i) + (ang_vel(i) * del_t(i))));
        ideal_y(i+1) = ideal_y(i) + ((net_vel(i)/ang_vel(i)) * cos(ideal_theta(i))) - ((net_vel(i)/ang_vel(i)) * cos(ideal_theta(i) + (ang_vel(i) * del_t(i))));
        ideal_theta(i+1) = ideal_theta(i) + (ang_vel(i) * del_t(i)) + (gamma * del_t(i));
    else
        ideal_x(i+1) = ideal_x(i) + (net_vel(i) * del_t(i));
        ideal_y(i+1) = ideal_y(i);
        ideal_theta(i+1) = ideal_theta(i);        
    end
    ideal_theta(i+1) = wrapToPi(ideal_theta(i+1));
end
