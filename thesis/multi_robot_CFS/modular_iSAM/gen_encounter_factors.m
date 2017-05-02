function encounter_factors = gen_encounter_factors(robot_activation_mask, var, robot_id, unit_fiducial_data, lmap_data)

import gtsam.*

global lmap_time_kD_tree robot_interaction_adjacency encounter_covariance;
global key_offset multi_robot current_factor_indices;

encounter_factors = [];

for i = 1:length(unit_fiducial_data.id)
    if unit_fiducial_data.id(i) == 0 || ...
       unit_fiducial_data.id(i) == 1 || ...
       unit_fiducial_data.id(i) == 2 || ...
       unit_fiducial_data.id(i) == 3
        if robot_activation_mask(unit_fiducial_data.id(i)+1)
            assert(unit_fiducial_data.id(i) + 1 ~= robot_id);
            k = 1;
            while 1
                nearest_target_lmap_index = knnsearch(lmap_time_kD_tree(robot_id).object, unit_fiducial_data.measurement_time, 'K', k);
                if nearest_target_lmap_index(end) <= var(unit_fiducial_data.id(i) + 1)
                    if abs(lmap_data(unit_fiducial_data.id(i) + 1).lmap(nearest_target_lmap_index(end)).measurement_time - ...
                           unit_fiducial_data.measurement_time) < 0.5
                        nearest_target_lmap_index = nearest_target_lmap_index(end);
                        break;
                    else
                        outer_continue = 1;
                        break;
                    end
                elseif abs(lmap_data(unit_fiducial_data.id(i) + 1).lmap(nearest_target_lmap_index(end)).measurement_time - ...
                           unit_fiducial_data.measurement_time) > 0.5
                       outer_continue = 1;
                       break;
                end
                k = k + 1;
            end
            
            if outer_continue
                outer_continue = 0;
                encounter_factors = [];
                continue;
            end
            
            nearest_target_lmap_index = knnsearch(lmap_time_kD_tree(robot_id).object, unit_fiducial_data.measurement_time);
            if isempty(lmap_data(unit_fiducial_data.id(i) + 1).lmap(nearest_target_lmap_index).pose)
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

            % The nearest lmap index in target maybe not even be explored yet  
            % in the target trajectory. So that might not have got the initial
            % guess. Although there is a small delay before the updation which
            % is given by isam_update_rate, it might fall near the border of
            % isam_update_rate and when calling the optimizer might not have
            % the initial guess provided by the target graph.
            offset_var = robot_id * key_offset(robot_id) + var(robot_id);
            offset_nearest_target_lmap_index = (unit_fiducial_data.id(i) + 1) * key_offset(unit_fiducial_data.id(i) + 1) + nearest_target_lmap_index;        
            encounter_factors = [encounter_factors BetweenFactorPose2(offset_var, ...
                                                                      offset_nearest_target_lmap_index, ...
                                                                      local_del_fiducial_pose, ...
                                                                      encounter_noise)];
            nearest_target_lmap_pose = lmap_data(unit_fiducial_data.id(i) + 1).lmap(nearest_target_lmap_index).pose;
            multi_robot.initial(robot_id).insert(offset_nearest_target_lmap_index, ...
                                                 Pose2(nearest_target_lmap_pose(1), ...
                                                       nearest_target_lmap_pose(2), ...
                                                       nearest_target_lmap_pose(3)));
            current_factor_indices{robot_id} = [current_factor_indices{robot_id}; offset_nearest_target_lmap_index];                                       
        else
            encounter_factors = [];
        end
    else
        encounter_factors = [];
        return;
    end
end
