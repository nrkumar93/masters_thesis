function lclosure_factor = gen_lclosure_factors(robot_id, var_1, var_2, pose_1, pose_2)

import gtsam.*

global loop_closure_covariance key_offset;

% ODOMETRY DEAD RECKONING CONSTRAINTS
% Calculating odometry using dead reckoning
[del_lc_x, del_lc_y, del_lc_theta] = odometry_difference(pose_1(1), pose_1(2), pose_1(3), ...
                                                      pose_2(1), pose_2(2), pose_2(3));
                                                          
% Delta time scaled Odometry Dead Reckoning covariance. 
lclosure_noise = noiseModel.Diagonal.Sigmas([loop_closure_covariance(robot_id, 1); ...
                                             loop_closure_covariance(robot_id, 5); ...
                                             loop_closure_covariance(robot_id, 9)]);

% Adding odometry contraints to graph
offset_var_1 = robot_id * key_offset(robot_id) + var_1;
offset_var_2 = robot_id * key_offset(robot_id) + var_2;
lclosure_factor = BetweenFactorPose2(offset_var_1, offset_var_2, Pose2(del_lc_x, del_lc_y, del_lc_theta), lclosure_noise);
