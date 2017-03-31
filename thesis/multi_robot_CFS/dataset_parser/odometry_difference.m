function [del_x, del_y, del_theta] = odometry_difference(x_1, y_1, theta_1, x_2, y_2, theta_2)

% Function to find the incremental change in processed odometry values.

del = rot(theta_1)' * [x_2 - x_1; y_2 - y_1];
del_x = del(1);
del_y = del(2);

theta_1 = wrapTo2Pi(theta_1);
theta_2 = wrapTo2Pi(theta_2);

del_theta = theta_2 - theta_1;
if del_theta > pi
    del_theta = del_theta - (2 * pi);
elseif del_theta < -pi
    del_theta = del_theta + (2 * pi);
end