function encounter_factors = gen_encounter_factors(robot_activation_mask, var, robot_id, unit_fiducial_data, lmap_data)

import gtsam.*

global lmap_time_kD_tree robot_interaction_adjacency encounter_covariance;
global key_offset multi_robot current_factor_indices blacklist_factor_indices;

encounter_factors = [];

for i = 1:length(unit_fiducial_data.id)
    if unit_fiducial_data.id(i) == 0 || ...
       unit_fiducial_data.id(i) == 1 || ...
       unit_fiducial_data.id(i) == 2 || ...
       unit_fiducial_data.id(i) == 3
        
        target_robot_id = unit_fiducial_data.id(i)+1;
        if robot_activation_mask(target_robot_id)
            assert(target_robot_id ~= robot_id);
            
            k = 1;
            outer_continue = 0;
            while 1
                nearest_target_lmap_index = knnsearch(lmap_time_kD_tree(target_robot_id).object, ...
                                                      unit_fiducial_data.measurement_time, ...
                                                      'K', ... 
                                                      k);
                
                target_lmap_data = lmap_data(target_robot_id).lmap(nearest_target_lmap_index(end));
                if ~isempty(target_lmap_data.pose)
                    if abs(target_lmap_data.measurement_time - unit_fiducial_data.measurement_time) < 0.5
                        break;
                    else
                        outer_continue = 1;
                        break;
                    end
                end
                k = k + 1;
            end
            
            if outer_continue
                outer_continue = 0;
                encounter_factors = [];
                continue;
            end
            
            local_del_fiducial_range = unit_fiducial_data.range(i);
            local_del_fiducial_bearing = unit_fiducial_data.bearing(i);
            local_del_fiducial_pose = Pose2(local_del_fiducial_range*cos(local_del_fiducial_bearing), ...
                                            local_del_fiducial_range*sin(local_del_fiducial_bearing), ...
                                            local_del_fiducial_bearing);

            % Updating robot interaction adjacency
            robot_interaction_adjacency(robot_id, target_robot_id) = ...
                robot_interaction_adjacency(robot_id, target_robot_id) + 1;        
            robot_interaction_adjacency(target_robot_id, robot_id) = ...
                robot_interaction_adjacency(robot_id, target_robot_id);

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
            
            offset_nearest_target_lmap_index = target_robot_id * key_offset(target_robot_id) + nearest_target_lmap_index;        
            
            nearest_target_lmap_pose = lmap_data(target_robot_id).lmap(nearest_target_lmap_index).pose;
            
            if offset_nearest_target_lmap_index < min(current_factor_indices{target_robot_id})
                encounter_factors = [encounter_factors BetweenFactorPose2(offset_var, ...
                                                          offset_nearest_target_lmap_index, ...
                                                          local_del_fiducial_pose, ...
                                                          encounter_noise)];
            elseif any(current_factor_indices{target_robot_id} == offset_nearest_target_lmap_index)
                if robot_id > target_robot_id
                    encounter_factors = [encounter_factors BetweenFactorPose2(offset_var, ...
                                                              offset_nearest_target_lmap_index, ...
                                                              local_del_fiducial_pose, ...
                                                              encounter_noise)];
                elseif robot_id < target_robot_id
                    encounter_factors = [encounter_factors BetweenFactorPose2(offset_var, ...
                                                              offset_nearest_target_lmap_index, ...
                                                              local_del_fiducial_pose, ...
                                                              encounter_noise)];
                                                          
                    if ~multi_robot.initial(robot_id).exists(offset_nearest_target_lmap_index)
                        multi_robot.initial(robot_id).insert(offset_nearest_target_lmap_index, ...
                                                             multi_robot.initial(target_robot_id).at(offset_nearest_target_lmap_index));
                                                         
                        multi_robot.initial(target_robot_id).erase(offset_nearest_target_lmap_index);
                    end                                                     
                end 
                    
            elseif offset_nearest_target_lmap_index > max(current_factor_indices{target_robot_id})
                encounter_factors = [encounter_factors BetweenFactorPose2(offset_var, ...
                                                          offset_nearest_target_lmap_index, ...
                                                          local_del_fiducial_pose, ...
                                                          encounter_noise)];

                if ~multi_robot.initial(robot_id).exists(offset_nearest_target_lmap_index)                                                      
                    multi_robot.initial(robot_id).insert(offset_nearest_target_lmap_index, ...
                                                         Pose2(nearest_target_lmap_pose(1), ...
                                                               nearest_target_lmap_pose(2), ...
                                                               nearest_target_lmap_pose(3)));
                                                           
                   blacklist_factor_indices{target_robot_id} = [blacklist_factor_indices{target_robot_id} offset_nearest_target_lmap_index]; 
                end                
            end                                                                  
            current_factor_indices{robot_id} = [current_factor_indices{robot_id}; offset_nearest_target_lmap_index];                                       
        end
    end
end
