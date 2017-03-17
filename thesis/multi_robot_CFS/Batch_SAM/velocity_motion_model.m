function [del_x, del_y, del_theta] = velocity_motion_model(lin_vel, ang_vel, theta, del_t)

% Function that containts the basic physics velocity model for a planar 3D
% (x,y,theta) mobile robot.

% The scale factor for translational velocity
beta = 4;
% The scale factor for rotational velocity
gamma = 3;

% Rotation and Tranlation update equations
if ang_vel ~= 0
    del_x = -((lin_vel*beta/ang_vel) * sin(theta)) + ...
            ((lin_vel*beta/ang_vel) * sin(theta + (ang_vel * del_t)));
    del_y = ((lin_vel*beta/ang_vel) * cos(theta)) - ...
            ((lin_vel*beta/ang_vel) * cos(theta + (ang_vel * del_t)));
    del_theta = gamma * (ang_vel * del_t);
% Pure rotation update equations
else 
    del_x = lin_vel * beta * del_t;
    del_y = 0;  
    del_theta = 0;
end
