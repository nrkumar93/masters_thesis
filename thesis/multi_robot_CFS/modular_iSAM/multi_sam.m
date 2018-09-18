function results = multi_sam(robot_activation_mask, multi_robot_mode, constraint_code, data, params)

import gtsam.*

global multi_robot key_offset all_factor_indices;

graph = [];
priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
for i = 1:length(robot_activation_mask)
    factor_graph = NonlinearFactorGraph;
    graph = [graph factor_graph];
    if i == 1
        graph(i).add(PriorFactorPose2((i * key_offset(i)) + 1, Pose2(0, 0, 0), priorNoise));
    elseif i == 2
        graph(i).add(PriorFactorPose2((i * key_offset(i)) + 1, Pose2(-1.22, 0.61, 0), priorNoise));
    elseif i == 3
        graph(i).add(PriorFactorPose2((i * key_offset(i)) + 1, Pose2(-2.44, 0, 0), priorNoise));
    elseif i == 4
        graph(i).add(PriorFactorPose2((i * key_offset(i)) + 1, Pose2(-3.66, 0.61, 0), priorNoise));
    end
%     graph(i).add(PriorFactorPose2((i * key_offset(i)) + 1, Pose2(0, 0, 3.142), priorNoise));
    initial_guess_module([], i, (i * key_offset(i)) + 1);
end

results = [];
for i = 1:length(robot_activation_mask)
    val = Values;
    results = [results val];
end

var_1 = ones(length(robot_activation_mask),1);
var_2 = zeros(length(robot_activation_mask),1);

frame_id = cell(length(robot_activation_mask),1);
for i = 1:length(robot_activation_mask)
    frame_id{i} = 1;
end

for i = 1:length(robot_activation_mask)
    if robot_activation_mask(i) ~= 0
        lmap_data(i).lmap = data(i).lmap;
    end
end

isam_update_counter = zeros(length(robot_activation_mask),1);
robot_completion_flag = zeros(length(robot_activation_mask),1);
cached_frames = [];

switch constraint_code
    case 123
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

%% ODOMETRY DEAD RECKONING CONSTRAINTS
                    odom_factor = gen_odom_factors(i, ...
                                                   var_1(i), ...
                                                   var_2(i), ...
                                                   data(i).odom(var_1(i),:), ...
                                                   data(i).odom(var_2(i),:), ...
                                                   data(i).odom_meas_t(var_2(i)) - data(i).odom_meas_t(var_1(i)));
                    graph(i).add(odom_factor);
                    multi_robot.graph(i).add(odom_factor);
%% LMAP CONSTRAINTS
                    lmap_factor = gen_lmap_factors(i, ...
                                                   var_1(i), ...
                                                   var_2(i), ...
                                                   data(i).lmap(var_1(i)).pose, ...
                                                   data(i).lmap(var_2(i)).pose, ...
                                                   [nan, 0.1146, 0.2003, nan]);
%                                                    data(i).lmap(var_2(i)).measurement_time - data(i).lmap(var_1(i)).measurement_time);
                    graph(i).add(lmap_factor);
                    multi_robot.graph(i).add(lmap_factor);
%% LASER SCAN MATCHING CONSTRAINTS                    

                    cached_frames = [cached_frames frame_id{i}];
                    frame_id{i} = scan_match_pattern_selector(params(i).key_frame_mode, ...
                                                              params(i).key_frame_rate, ...
                                                              var_1(i), ...
                                                              frame_id{i});                                                          
                    scan_matching_factors  = gen_scan_match_factors(params(i).scan_matching_mode, ...
                                                                     params(i).key_frame_rate, ...
                                                                     i, ...
                                                                     frame_id{i}, ...
                                                                     var_2(i), ...
                                                                     data(i).laser(frame_id{i}).range, ...
                                                                     data(i).laser(var_2(i)).range, ...
                                                                     data(i).lmap(frame_id{i}).pose, ...
                                                                     data(i).lmap(var_2(i)).pose, ...
                                                                     data(i).laser(frame_id{i}).measurement_time, ...
                                                                     data(i).laser(var_2(i)).measurement_time);
                    for j = 1:length(scan_matching_factors)
                        graph(i).add(scan_matching_factors(j));
                        multi_robot.graph(i).add(scan_matching_factors(j));
                    end
%% FIDUCIAL CONSTRAINTS                     
                    fiducial_factors = gen_fiducial_factors(var_1(i), ...
                                                            i, ...
                                                            data(i).fiducials(var_1(i)), ...
                                                            results(i));
                    for j = 1:length(fiducial_factors)
                        graph(i).add(fiducial_factors(j));
                        multi_robot.graph(i).add(fiducial_factors(j));
                    end
%% LOOP CLOSURE CONSTRAINTS
%                     [~, loc] = ismember(var_1(i), data(i).closures(:,1));
%                     if loc ~= 0
%                         lclosure_factors = gen_lclosure_factors(i, ...
%                                                                 var_1(i), ...
%                                                                 data(i).closures(loc,2), ...
%                                                                 data(i).lmap(var_1(i)).pose, ...
%                                                                 data(i).lmap(data(i).closures(loc,2)).pose);
%                         graph(i).add(lclosure_factors);
%                         multi_robot.graph(i).add(lclosure_factors);                        
%                     end
%% ENCOUNTER CONSTRAINTS
                    if ~isempty(data(i).fiducials(var_1(i)))    
                        encounter_factors = gen_encounter_factors(robot_activation_mask, ...
                                                                  var_1, ...
                                                                  i, ...
                                                                  data(i).fiducials(var_1(i)), ...
                                                                  lmap_data);
                        for j = 1:length(encounter_factors)
                            multi_robot.graph(i).add(encounter_factors(j));
                            
                        end                        
                    end
                    continue;
%% INITIAL GUESS
                    initial_guess_module(data(i), ...
                                         i, ...
                                         var_1(i), ...
                                         var_2(i), ...
                                         'lmap');

%% OPTIMIZATION
                    isam_update_counter(i) = isam_update_counter(i) + 1;
                    if isam_update_counter(i) >= params(i).isam_update_rate && var_1(i) > params(i).batch_update_size
                        if params(i).batch_initialization
                            results = optimize('batch', ...
                                               multi_robot_mode, ...
                                                  i, ...
                                                  robot_activation_mask, ...
                                                  var_2(i), ...
                                                  graph(i), ...
                                                  results);
                            params(i).batch_initialization = false;
                        end
                        results = optimize('incremental', ...
                                            multi_robot_mode, ...
                                              i, ...
                                              robot_activation_mask, ...
                                              var_2(i), ...
                                              graph(i), ...
                                              results);
                             
                        graph(i) = NonlinearFactorGraph;
                        isam_update_counter(i) = 0;
                    end
                    all_factor_indices{i} = [all_factor_indices{i} var_1(i)];
                end
            end
            var_1 = var_2;
        end
    graph_fuser(robot_activation_mask,data,[])
end
