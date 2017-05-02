function result = optimize(mode, robot_id, robot_activation_mask, var_2, graph, result)

import gtsam.*

global multi_robot robot_interaction_adjacency;
global isam initial current_factor_indices key_offset;
global init_x init_y init_theta;

persistent batch_initialization visit_counter;
if isempty(batch_initialization)
    batch_initialization = zeros(length(initial),1);
end
if isempty(find(visit_counter == robot_id,1))
    visit_counter = [visit_counter robot_id];
    visit_counter = sort(visit_counter);
end

if isequal(mode, 'batch')
    batchOptimizer = LevenbergMarquardtOptimizer(graph, initial(robot_id));
    initial(robot_id) = batchOptimizer.optimize();
    batch_initialization(robot_id) = 1;
end

isam(robot_id).update(graph, initial(robot_id));
result(robot_id) = isam(robot_id).calculateEstimate();

var_2 = robot_id * key_offset(robot_id) + var_2;                
init_x(robot_id) = result(robot_id).at(var_2).x;
init_y(robot_id) = result(robot_id).at(var_2).y;
init_theta(robot_id) = result(robot_id).at(var_2).theta;

% for i = current_factor_indices{robot_id}'
%     % The first operand of the && operation in both if and elseif looks
%     % redundant because the variable is added to "robot_id" index of
%     % current_factor_indices only when an initial value is set for the same
%     % variable in "robot_id" index of multi_robot.initial. So all the
%     % variables in current_factor_indices(robot_id) should by definition
%     % exist in multi_robot.initial(robot_id).
%     %
%     % ASSUMPTION: key_offset for all the robots are SAME.
%     if multi_robot.initial(robot_id).exists(i) && result(robot_id).exists(i)
%         multi_robot.initial(robot_id).update(i,result(robot_id).at(i));
%     elseif multi_robot.initial(robot_id).exists(i) && result(floor(i/key_offset(robot_id))).exists(i)
%         multi_robot.initial(robot_id).update(i,result(floor(i/key_offset(robot_id))).at(i));
%     end
% end

if isequal(find(robot_activation_mask == 1), visit_counter) && isequal(batch_initialization', robot_activation_mask)
    
%     for j = find(robot_activation_mask == 1)
%         for i = current_factor_indices{j}'
%             if multi_robot.initial(j).exists(i)
%                 if result(j).exists(i)
%                     multi_robot.initial(j).update(i,result(j).at(i));
%                 elseif result(floor(i/key_offset(j))).exists(i)
%                     if j > floor(i/key_offset(j))
%                         if i >= min(current_factor_indices{floor(i/key_offset(j))}) && ...
%                            any(current_factor_indices{floor(i/key_offset(j))} == i)
%                             continue;
%                         elseif i >= min(current_factor_indices{floor(i/key_offset(j))}) && ...
%                                ~any(current_factor_indices{floor(i/key_offset(j))} == i)
%                             error('The target key seen from source not present in initial guess of target');
%                         end
%                     else
%                         % Could be optimized. Redundant else case below. If
%                         % could also be removed and can be written without
%                         % if else. 
%                         if any(current_factor_indices{floor(i/key_offset(j))} == i)
%                             current_factor_indices{floor(i/key_offset(j))}(current_factor_indices{floor(i/key_offset(j))} == i) = [];
%                             if multi_robot.initial(floor(i/key_offset(j))).exists(i)
%                                 multi_robot.initial(floor(i/key_offset(j))).erase(i)
%                             end
%                             if multi_robot.initial(j).exists(i)
%                                 multi_robot.initial(j).update(i,result(floor(i/key_offset(j))).at(i));
%                             else
%                                 multi_robot.initial(j).add(i,result(floor(i/key_offset(j))).at(i));
%                             end
%                         else
%                             current_factor_indices{floor(i/key_offset(j))}(current_factor_indices{floor(i/key_offset(j))} == i) = [];
%                             if multi_robot.initial(floor(i/key_offset(j))).exists(i)
%                                 multi_robot.initial(floor(i/key_offset(j))).erase(i)
%                             end
%                         end
%                     end
%                 end
%             end
%         end
%     end
    
    % ASSUMPTION: key_offset for all the robots are SAME.
    for j = find(robot_activation_mask == 1)
        for i = current_factor_indices{j}'
            if i < 1000000
                k = floor(i/key_offset(j));     % Target robot index
                if j == k
                    if multi_robot.initial(j).exists(i)
                        if result(j).exists(i)
                            multi_robot.initial(j).update(i,result(j).at(i));
                        end
                    else
                        error('Key present in current factor indices but not initialized in respective multi robot initials.');
                    end
                else
                    if j > k
                        assert(multi_robot.initial(k).exist(i) || i <= min(current_factor_indices{k}));
                        if multi_robot.initial(j).exist(i)
                            current_factor_indices{j}(current_factor_indices{j} == i) = [];
                            multi_robot.initial(j).erase(i);
                        end
                    else
                        if any(current_factor_indices{k} == i)
                            assert(multi_robot.initial(k).exist(i));
                            assert(multi_robot.initial(j).exist(i));
                            multi_robot.initial(j).update(i, result(k).at(i));
                        else
                            assert(i <= min(current_factor_indices{k}));
                            assert(multi_robot.initial(j).exist(i));
                        end
                    end
                end
            else 
                continue;
            end
        end
    end
    
    
    for i = 1:size(robot_interaction_adjacency,1)
        for j = 1:i-1
            if i ~= j && robot_interaction_adjacency(i,j) > 0
                multi_robot.isam.update(multi_robot.graph(i), multi_robot.initial(i));
                multi_robot.isam.update(multi_robot.graph(j), multi_robot.initial(j));
                multi_robot.graph(i) = NonlinearFactorGraph;
                multi_robot.graph(j) = NonlinearFactorGraph;
                multi_robot.initial(i) = Values;
                multi_robot.initial(j) = Values;
                robot_interaction_adjacency(i,j) = 0;
                robot_interaction_adjacency(j,i) = 0;
                current_factor_indices{i} = [];
                current_factor_indices{j} = [];
            end
        end
    end
    visit_counter = [];
end
initial(robot_id) = Values;

