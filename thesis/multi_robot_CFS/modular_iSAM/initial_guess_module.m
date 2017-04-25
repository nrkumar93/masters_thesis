function [] = initial_guess_module(mode, unit_data, robot_id, var_1, var_2)

global initial;
global init_x init_y init_theta;

if ~isequal(mode, 'lmap') || ~isequal(mode, 'odom') || ~isequal(mode, 'vel')
    error('No matching source for initial guess')
end

if isequal(mode, 'lmap')
    [init_del_x, init_del_y, init_del_theta] = odometry_difference(unit_data.lmap(var_1).pose(1), ...
                                                        unit_data.lmap(var_1).pose(2), ...
                                                        unit_data.lmap(var_1).pose(3), ...
                                                        unit_data.lmap(var_2).pose(1), ...
                                                        unit_data.lmap(var_2).pose(2), ...
                                                        unit_data.lmap(var_2).pose(3));

elseif isequal(mode, 'odom')
    [init_del_x, init_del_y, init_del_theta] = odometry_difference(unit_data.odom(var_1).pose(1), ...
                                                        unit_data.odom(var_1).pose(2), ...
                                                        unit_data.odom(var_1).pose(3), ...
                                                        unit_data.odom(var_2).pose(1), ...
                                                        unit_data.odom(var_2).pose(2), ...
                                                        unit_data.odom(var_2).pose(3));

elseif isequal(mode, 'vel')          
    if var_2 - var_1 ~= 1
        error('Initial guess through velocity motion model not supported for multiple timesteps.');
    end
    [init_del_x, init_del_y, init_del_theta] = velocity_motion_model(unit_data.velocity(var_1,1), unit_data.velocity(var_1,2), ...
                                                                     unit_data.lmap(var_1).pose(3), unit_data.del_t(var_1));
    
end

init_x(robot_id) = init_x(robot_id) + init_del_x;
init_y(robot_id) = init_y(robot_id) + init_del_y;
init_theta(robot_id) = init_theta(robot_id) + init_del_theta;

initial(robot_id).insert(var_2, Pose2(init_x(robot_id), init_y(robot_id), init_theta(robot_id)));

