function results = multi_sam(robot_activation_mask, constraint_code, data, params)

graph = NonlinearFactorGraph;
graph = [graph graph graph graph];

mega_graph = NonlinearFactorGraph;

results = Values;
results = [results results results results];

robot_interaction_adjacency = zeros(length(robot_activation_mask), length(robot_activation_mask));

var_1 = ones(length(robot_activation_mask));
var_2 = zeros(length(robot_activation_mask));

frame_id = ones(length(robot_activation_mask));

switch constraint_code
    case 123
        while 1
            for i = 1:length(robot_activation_mask)
                if robot_activation_mask(i) ~= 0
                    while var_1(i) <= data(i).data_size - 1
                        var_2(i) = var_1(i) + 1;
                        while var_2(i) <= data(i).data_size
                            if ~isempty(data(i).lmap(var_2(i)))
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
                    

%% ODOMETRY DEAD RECKONING CONSTRAINTS                    
                    graph(i).add(gen_odom_factors(var_1(i), ...
                                                  var_2(i), ...
                                                  data(i).odom(var_1(i),:), ...
                                                  data(i).odom(var_2(i),:), ...
                                                  data(i).del_t(var_2(i)) - data(i).del_t(var_1(i))));                
%% LMAP CONSTRAINTS                                              
                    graph(i).add(gen_lmap_factors(var_1(i), ...
                                                  var_2(i), ...
                                                  data(i).lmap(var_1(i)).pose, ...
                                                  data(i).lmap(var_2(i)).pose, ...
                                                  data(i).del_t(var_2(i)) - data(i).del_t(var_1(i))));
%% LASER SCAN MATCHING CONSTRAINTS                    
                    frame_id(i) = scan_match_pattern_selector(params(i).key_frame_mode, ...
                                                              params(i).key_frame_rate, ...
                                                              var_i(i), ...
                                                              frame_id(i));                                                          
                    scan_matching_factors  = gen_scan_match_factors(params(i).scan_matching_mode, ...
                                             params(i).key_frame_rate, ...
                                             var_1(i), ...
                                             var_2(i), ...
                                             data(i).laser(var_1(i)).range, ...
                                             data(i).laser(var_2(i)).range);
                    for j = scan_matching_factors
                        graph(i).add(j);
                    end
%% FIDUCIAL CONSTRAINTS                     
                    fiducial_factors = gen_fiducial_factors(i, ...
                                                            var_1(i), ...
                                                            data(i).fiducials(var_1(i)), ...
                                                            results(i));
                    for j = fiducial_factors
                        graph(i).add(j);
                    end

                end
            end
        end
end