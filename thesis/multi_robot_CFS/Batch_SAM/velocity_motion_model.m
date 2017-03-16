function [del_x, del_y, del_theta] = velocity_motion_model(lin_vel, ang_vel, theta, del_t)

beta = 4;
gamma = 1;

if ang_vel ~= 0
    del_x = -((lin_vel*beta/ang_vel) * sin(theta)) + ...
            ((lin_vel*beta/ang_vel) * sin(theta + (ang_vel * del_t)));
    del_y = ((lin_vel*beta/ang_vel) * cos(theta)) - ...
            ((lin_vel*beta/ang_vel) * cos(theta + (ang_vel * del_t)));
    del_theta = gamma * (ang_vel * del_t);
else 
    del_x = net_vel * beta * del_t;
    del_y = ~;

end
