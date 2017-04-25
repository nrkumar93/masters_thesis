function [] = setup_globals(robot_activation_mask, robot_id, unit_data)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FOR NOW DO NOT CALL THIS FUNCTION MORE THAN ONCE.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global robot_interaction_adjacency lmap_time_kD_tree isam;
global initial;
global init_x init_y init_theta;

robot_interaction_adjacency = zeros(length(robot_activation_mask), length(robot_activation_mask));

if isempty(initial)
    initial = gtsam.Values;
    initial = [initial initial initial initial];
    init_x = zeros(4,1);
    init_y = zeros(4,1);
    init_theta = zeros(4,1);
end

if isempty(initial)
    isam = ISAM2;
    isam = [isam isam isam isam];
end

lmap_time_pool = zeros(length(unit_data),1);
for i = 1:length(unit_data)
    if ~isempty(unit_data(i).measurement_time)
        lmap_time_pool(i) = unit_data(i).measurement_time;
    end
end

if robot_activation_mask(robot_id) ~= 0
    lmap_time_kD_tree(robot_id).object = createns(lmap_time_pool);
end
