function [ideal_x, ideal_y, ideal_theta] = ideal_model(datamat)

robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

gamma = 3 ;
beta = 4;

meas_t = [];
meas_t = [meas_t, robot.odom.measurement_time];
del_t = diff(meas_t);

vels = [];
vels = [vels, robot.odom.velocity];

net_vel = vels(1:3:end);
ang_vel = vels(3:3:end);

% The following 2 lines are used when the optimal velocities are calculated
% using the reverse of this velocity model.
% net_vel = opt_lin_vel;
% ang_vel = opt_ang_vel;

ideal_x = zeros(length(del_t) + 1, 1);
ideal_y = zeros(length(del_t) + 1, 1);
ideal_theta = zeros(length(del_t) + 1, 1);

for i = 1:length(del_t)
    if ang_vel(i) ~= 0
        ideal_x(i+1) = ideal_x(i) - ((net_vel(i)*beta/ang_vel(i)) * sin(ideal_theta(i))) + ...
            ((net_vel(i)*beta/ang_vel(i)) * sin(ideal_theta(i) + (ang_vel(i) * del_t(i))));
        ideal_y(i+1) = ideal_y(i) + ((net_vel(i)*beta/ang_vel(i)) * cos(ideal_theta(i))) - ...
            ((net_vel(i)*beta/ang_vel(i)) * cos(ideal_theta(i) + (ang_vel(i) * del_t(i))));
%         if abs(ang_vel(i)) > 0.2
%             gamma = 2.5;
%         else
%             gamma = 5.5;
%         end
        ideal_theta(i+1) = ideal_theta(i) + gamma * (ang_vel(i) * del_t(i));
    else
        ideal_x(i+1) = ideal_x(i) + (net_vel(i)*beta * del_t(i));
        ideal_y(i+1) = ideal_y(i);
        ideal_theta(i+1) = ideal_theta(i);
    end
    
    ideal_theta(i+1) = wrapToPi(ideal_theta(i+1));
end
