function scan_match_factor = gen_scan_match_factors(mode ,keyframe_rate ,var_1, var_2, range_1, range_2, init_1, init_2)

if nargin == 6
    assert(mode == 1 || mode == 2);
end

global scan_matching_covariance;

scan_match_factor = [];

if mode == 1
    for i = 1:length(var_1)
        [scan_match_R, scan_match_T] = scan_matcher(range_1(i,:), range_2);

        if ~isempty(scan_match_R) && ~isempty(scan_match_T)
            [odom_frame_x, odom_frame_y, ~] =  odometry_difference(init_1(i,1), ...
                                                                   init_1(i,2), ...
                                                                   init_1(i,3), ...
                                                                   init_2(1), ...
                                                                   init_2(2), ...
                                                                   init_2(3));

            disparity = abs(norm([odom_frame_x, odom_frame_y]) - norm(scan_match_T));
            scan_matching_theta = acos(scan_match_R(1));
        
            if disparity < (0.01 * keyframe_rate) || norm(scan_match_T) < (keyframe_rate * 0.1)
                scan_match_factor = [scan_match_factor, BetweenFactorPose2(var_1(i), var_2, ...
                Pose2(scan_match_T(1), scan_match_T(2), scan_matching_theta), scan_matching_covariance)]; 
            end
        end
    end
elseif mode == 2
    for i = 1:length(var_1)
        [scan_match_R, scan_match_T] = csm_scan_matcher(range_1(i,:), range_2);
        if ~isempty(scan_match_R) && ~isempty(scan_match_T)
            [odom_frame_x, odom_frame_y, ~] =  odometry_difference(init_1(i,1), ...
                                                                   init_1(i,2), ...
                                                                   init_1(i,3), ...
                                                                   init_2(1), ...
                                                                   init_2(2), ...
                                                                   init_2(3));

            disparity = abs(norm([odom_frame_x, odom_frame_y]) - norm(scan_match_T));
            scan_matching_theta = acos(scan_match_R(1));
        
            if disparity < (0.01 * keyframe_rate) || norm(scan_match_T) < (keyframe_rate * 0.1)
                scan_match_factor = [scan_match_factor BetweenFactorPose2(var_1(i), var_2, ...
                Pose2(scan_match_T(1), scan_match_T(2), scan_matching_theta), scan_matching_covariance)]; 
            end
        end
    end
elseif mode == 3
    dummy_time_1 = 1;
    dummy_time_2 = 2;
    for i = 1:length(var_1)
        [csm_init_x, csm_init_y, csm_init_theta] = odometry_difference(init_1(i,1), ...
                                                                       init_1(i,2), ...
                                                                       init_1(i,3), ...
                                                                       init_2(1), ...
                                                                       init_2(2), ...
                                                                       init_2(3));

        [scan_match_R, scan_match_T, scan_matching_noise, scan_matching_theta] = ...
            fast_csm_scan_matcher(dummy_time_1, range_1(i), ...
            dummy_time_2, range_2, [csm_init_x, csm_init_y, csm_init_theta]);

        if ~isempty(scan_match_R) && ~isempty(scan_match_T)
            scan_matching_noise = noiseModel.Gaussian.Covariance(reshape(scan_matching_noise, [3 3]));
            disparity = abs(norm([csm_init_x, csm_init_y]) - norm(scan_match_T));
            if disparity < (0.01 * keyframe_rate) || norm(scan_match_T) < (keyframe_rate * 0.1)
                scan_match_factor = [scan_match_factor BetweenFactorPose2(var_1(i), var_2, ...
                        Pose2(scan_match_T(1), scan_match_T(2), scan_matching_theta), scan_matching_noise)];        
            end
        end
    end    
end

