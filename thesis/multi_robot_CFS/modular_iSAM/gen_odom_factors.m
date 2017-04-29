function odom_factor = gen_odom_factors(robot_id, var_1, var_2, pose_1, pose_2, del_t)

import gtsam.*

global odometry_covariance_per_time_ratio key_offset;

% ODOMETRY DEAD RECKONING CONSTRAINTS
% Calculating odometry using dead reckoning
[o_del_x, o_del_y, o_del_theta] = odometry_difference(pose_1(1), pose_1(2), pose_1(3), ...
                                                      pose_2(1), pose_2(2), pose_2(3));
                                                          
% Delta time scaled Odometry Dead Reckoning covariance. 
odom_noise = noiseModel.Diagonal.Sigmas([odometry_covariance_per_time_ratio(1); ...
                                         odometry_covariance_per_time_ratio(5); ...
                                         odometry_covariance_per_time_ratio(9)] * (del_t^2));


% Adding odometry contraints to graph
offset_var_1 = robot_id * key_offset(robot_id) + var_1;
offset_var_2 = robot_id * key_offset(robot_id) + var_2;
odom_factor = BetweenFactorPose2(offset_var_1, offset_var_2, Pose2(o_del_x, o_del_y, o_del_theta), odom_noise);
