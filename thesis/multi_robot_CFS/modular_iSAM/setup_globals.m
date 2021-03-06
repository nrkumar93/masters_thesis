function [] = setup_globals(robot_activation_mask, data, params)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FOR NOW DO NOT CALL THIS FUNCTION MORE THAN ONCE.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
import gtsam.*

global robot_interaction_adjacency lmap_time_kD_tree isam;
global multi_robot;
global initial;
global current_factor_indices;
global key_offset;
global blacklist_factor_indices;
global all_factor_indices;
global init_x init_y init_theta;
global fused_isam;

robot_interaction_adjacency = zeros(length(robot_activation_mask), length(robot_activation_mask));

if isempty(key_offset)
    key_offset = zeros(length(robot_activation_mask),1);
    for i = 1:length(robot_activation_mask)
        if robot_activation_mask(i) ~= 0
            key_offset(i) = params(i).key_offset;
        end
    end
end

if isempty(multi_robot)
    multi_robot.graph = [];
    for i = 1:length(robot_activation_mask)
        multi_graph = NonlinearFactorGraph;
        multi_robot.graph = [multi_robot.graph multi_graph];
    end
    
    multi_robot.isam = ISAM2;
    
    multi_robot.initial = [];
    for i = 1:length(robot_activation_mask)
        multi_initial = Values;
        multi_robot.initial = [multi_robot.initial multi_initial];
    end
end

if isempty(isam)
    isam = [];
    for i = 1:length(robot_activation_mask)
        isam_obj = ISAM2;
        isam = [isam isam_obj];
    end
end

if isempty(fused_isam)
    fused_isam = ISAM2;
end

priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
if isempty(initial)
    initial = [];
    for i = 1:length(robot_activation_mask)
        val = Values;
        initial = [initial val];
    end
    
    init_x = zeros(4,1);
    init_y = zeros(4,1);
    init_theta = 3.142 * ones(4,1);

    for i = 1:length(robot_activation_mask)
        if robot_activation_mask(i) ~= 0
            if i == 1
                multi_robot.graph(i).add(PriorFactorPose2((i * key_offset(i)) + 1, Pose2(0, 0, 0), priorNoise));
                init_x(i) = 0;
                init_y(i) = 0;
                init_theta(i) = 0;
            elseif i == 2
                multi_robot.graph(i).add(PriorFactorPose2((i * key_offset(i)) + 1, Pose2(-1.22, 0.61, 0), priorNoise));
                init_x(i) = -1.22;
                init_y(i) = 0.61;
                init_theta(i) = 0;
            elseif i == 3
                multi_robot.graph(i).add(PriorFactorPose2((i * key_offset(i)) + 1, Pose2(-2.44, 0, 0), priorNoise));
                init_x(i) = -2.44;
                init_y(i) = 0;
                init_theta(i) = 0;
            elseif i == 4
                multi_robot.graph(i).add(PriorFactorPose2((i * key_offset(i)) + 1, Pose2(-3.66, 0.61, 0), priorNoise));
                init_x(i) = -3.66;
                init_y(i) = 0.61;
                init_theta(i) = 0;
            end
        end
    end
end

if isempty(current_factor_indices)
    current_factor_indices = cell(length(robot_activation_mask),1);
end

if isempty(blacklist_factor_indices)
    blacklist_factor_indices = cell(length(robot_activation_mask),1);
end

if isempty(all_factor_indices)
    all_factor_indices = cell(length(robot_activation_mask),1);
end

for i = 1:length(robot_activation_mask)
    if robot_activation_mask(i) ~= 0
    lmap_time_pool = zeros(length(data(i).lmap),1);
    for j = 1:length(data(i).lmap)
        if ~isempty(data(i).lmap(j).measurement_time)
            lmap_time_pool(j) = data(i).lmap(j).measurement_time;
        end
    end
    lmap_time_kD_tree(i).object = createns(lmap_time_pool);
    end
end
