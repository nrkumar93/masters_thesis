function graph_fuser(robot_activation_mask, data, params)

import gtsam.*

global fused_isam

graph = NonlinearFactorGraph;

var_1 = ones(length(robot_activation_mask),1);
var_2 = zeros(length(robot_activation_mask),1);
robot_completion_flag = zeros(length(robot_activation_mask),1);

for i = 1:length(robot_activation_mask)
    if robot_activation_mask(i) ~= 0
        lmap_data(i).lmap = data(i).lmap;
    end
end

while 1

    for i = 1:length(robot_activation_mask)
        if robot_activation_mask(i) ~= 0
            while var_1(i) <= data(i).data_size - 1
                var_2(i) = var_1(i) + 1;
                while var_2(i) <= data(i).data_size
                    if ~isempty(data(i).lmap(var_2(i)).pose)
                        break;
                    end
                    var_2(i) = var_2(i) + 1;
                end
                break;                                                
            end
        end
    end

    for i = 1:length(robot_activation_mask)
        if robot_activation_mask(i) ~= 0
            if var_1(i) == var_2(i)
                robot_completion_flag(i) = 1;
            end
        end
    end

    if isequal(robot_activation_mask, robot_completion_flag')
        break;
    end

    for i = 1:length(robot_activation_mask)
        if robot_activation_mask(i) ~= 0

            if robot_completion_flag(i) == 1
                continue;
            end

    %% ENCOUNTER CONSTRAINTS
            if ~isempty(data(i).fiducials(var_1(i)))    
                encounter_factors = stumble_factors(robot_activation_mask, ...
                                                    var_1, ...
                                                    i, ...
                                                    data(i).fiducials(var_1(i)), ...
                                                    lmap_data);
                for j = 1:length(encounter_factors)
                    graph.add(encounter_factors(j));

                end                        
            end                    
        end
    end
    var_1 = var_2;
end
init = Values;
fused_isam.update(graph, init);
end

function encounter_factors = stumble_factors(robot_activation_mask, var, robot_id, unit_fiducial_data, lmap_data)

    import gtsam.*

    global lmap_time_kD_tree key_offset encounter_covariance;
    
    encounter_factors = [];
    
    for i = 1:length(unit_fiducial_data.id)
        if unit_fiducial_data.id(i) == 0 || ...
           unit_fiducial_data.id(i) == 2 || ...
           unit_fiducial_data.id(i) == 3
%            unit_fiducial_data.id(i) == 1 || ...

            target_robot_id = unit_fiducial_data.id(i) + 1;
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
                    encounter_factors = [];
                    continue;
                end

                encounter_noise = noiseModel.Diagonal.Sigmas([encounter_covariance(robot_id, 1); ...
                                                              encounter_covariance(robot_id, 5); ...
                                                              encounter_covariance(robot_id, 9)]);        
                
                local_del_fiducial_range = unit_fiducial_data.range(i);
                local_del_fiducial_bearing = unit_fiducial_data.bearing(i);
                local_del_fiducial_pose = Pose2(local_del_fiducial_range*cos(local_del_fiducial_bearing), ...
                                                local_del_fiducial_range*sin(local_del_fiducial_bearing), ...
                                                local_del_fiducial_bearing);
                
                offset_var = robot_id * key_offset(robot_id) + var(robot_id);

                offset_nearest_target_lmap_index = target_robot_id * key_offset(target_robot_id) + nearest_target_lmap_index;        
                    
                encounter_factors = [encounter_factors BetweenFactorPose2(offset_var, ...
                                                          offset_nearest_target_lmap_index, ...
                                                          local_del_fiducial_pose, ...
                                                          encounter_noise)];
                
                
                
            end
        end
    end

end
