function encounter_factors = gen_encounter_factors(var, robot_id, unit_fiducial_data, lmap_data)

global lmap_time_kD_tree robot_interaction_adjacency encounter_covariance;

encounter_factors = [];

for i = 1:length(unit_fiducial_data.id)
    if unit_fiducial_data.id(i) == 0 || ...
       unit_fiducial_data.id(i) == 1 || ...
       unit_fiducial_data.id(i) == 2 || ...
       unit_fiducial_data.id(i) == 3
        
        assert(unit_fiducial_data.id(i) + 1 ~= robot_id);
        nearest_target_lmap_index = knnsearch(lmap_time_kD_tree(robot_id).object, unit_fiducial_data.measurement_time);
        if isempty(lmap_data(unit_fiducial_data.id(i)).lmap(nearest_target_lmap_index).pose)
            encounter_factors = [];
            fprintf('lmap entry is empty at %d\n', nearest_target_lmap_index);
            return;
        else
            local_del_fiducial_range = unit_fiducial_data.range(i);
            local_del_fiducial_bearing = unit_fiducial_data.bearing(i);
            local_del_fiducial_pose = Pose2(local_del_fiducial_range*cos(local_del_fiducial_bearing), ...
                                            local_del_fiducial_range*sin(local_del_fiducial_bearing), ...
                                            local_del_fiducial_bearing);
        end
            
        % Updating robot interaction adjacency
        robot_interaction_adjacency(robot_id, unit_fiducial_data.id(i) + 1) = ...
            robot_interaction_adjacency(robot_id, unit_fiducial_data.id(i) + 1) + 1;        
        robot_interaction_adjacency(unit_fiducial_data.id(i) + 1, robot_id) = ...
            robot_interaction_adjacency(robot_id, unit_fiducial_data.id(i) + 1);
        
        encounter_noise = noiseModel.Diagonal.Sigmas([encounter_covariance(1); ...
                                                      encounter_covariance(5); ...
                                                      encounter_covariance(9)]);        
        
        encounter_factors = [encounter_factors BetweenFactorPose2(var, nearest_target_lmap_index, local_del_fiducial_pose, encounter_noise)];
    else
        encounter_factors = [];
        return;
    end
end
