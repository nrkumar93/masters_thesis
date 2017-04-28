function lclosure_factor = gen_lclosure_factors(var_1, var_2, pose_1, pose_2)

import gtsam.*

global loop_closure_covariance

% ODOMETRY DEAD RECKONING CONSTRAINTS
% Calculating odometry using dead reckoning
[del_lc_x, del_lc_y, del_lc_theta] = odometry_difference(pose_1(1), pose_1(2), pose_1(3), ...
                                                      pose_2(1), pose_2(2), pose_2(3));
                                                          
% Delta time scaled Odometry Dead Reckoning covariance. 
lclosure_noise = noiseModel.Diagonal.Sigmas([loop_closure_covariance(1); ...
                                             loop_closure_covariance(5); ...
                                             loop_closure_covariance(9)]);

% Adding odometry contraints to graph
lclosure_factor = BetweenFactorPose2(var_1, var_2, Pose2(del_lc_x, del_lc_y, del_lc_theta), lclosure_noise);
