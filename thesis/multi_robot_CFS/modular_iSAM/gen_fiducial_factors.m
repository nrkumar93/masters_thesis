function fiducial_factor = gen_fiducial_factors(var, robot_id, fiducial_data, current_result)

global fiducial_covariance;
global initial;
global init_x init_y init_theta;

fiducial_factor = [];

for ele = 1:length(fiducial_data.id)
    if fiducial_data.id(ele) > 3 %|| robot.fiducial(i).id(j) == -1
        [fid_del_x, fid_del_y, fid_del_theta] = ...
            fiducial_processor(fiducial_data.range(ele), fiducial_data.bearing(ele));

        del_fid = Pose2(fid_del_x, fid_del_y, fid_del_theta);
        landmark_key = symbol('l', fiducial_data.id(ele));

        % Fiducial covariance. Very low to assert almost zero error.
        fiducial_noise = noiseModel.Diagonal.Sigmas([fiducial_covariance(1); ...
                                                     fiducial_covariance(5); ...
                                                     fiducial_covariance(9)]);

        fiducial_factor = [fiducial_factor BetweenFactorPose2(var, landmark_key, del_fid, fiducial_noise)];
    end
    
    if ~initial(robot_id).exists(landmark_key) && ~current_result.exists(landmark_key)
        fid_init_point2 = (([init_x(robot_id); init_y(robot_id)]) + rot(init_theta(robot_id)) * [fid_del_x; fid_del_y])';
        initial(robot_id).insert(landmark_key, Pose2(fid_init_point2(1), fid_init_point2(2), 0));
    elseif current_result.exists(landmark_key)
        estimated_global_pose2 = Pose2(init_x(robot_id), init_y(robot_id), init_theta(robot_id));
        graph.add(BetweenFactorPose2(var, landmark_key, ...
            estimated_global_pose2.between(current_result.at(landmark_key)), noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01])));                            
    end
    
end

