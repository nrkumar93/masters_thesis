clear;

source_robot_id = 3;
target_robot_id = 2;

%% Load the saved robot specific dataset.
source_datamat = strcat('../data/dataset_robot', string(source_robot_id), '.mat');
target_datamat = strcat('../data/dataset_robot', string(target_robot_id), '.mat');

source_robot = load(source_datamat);
source_robot = getfield(source_robot, char(fieldnames(source_robot)));
target_robot = load(target_datamat);
target_robot = getfield(target_robot, char(fieldnames(target_robot)));

if source_robot.odom(1).robot_id == 1
    source_data_start_point = 3500;
    source_data_end_point = length(source_robot.odom);
elseif source_robot.odom(1).robot_id == 2
    source_data_start_point = 3500;
    source_data_end_point = 13000;
elseif source_robot.odom(1).robot_id == 3
    source_data_start_point = 4000;
    source_data_end_point = length(source_robot.odom);
elseif source_robot.odom(1).robot_id == 4
    source_data_start_point = 3500;
    source_data_end_point = length(source_robot.odom);
end

if target_robot.odom(1).robot_id == 1
    target_data_start_point = 3500;
    target_data_end_point = length(target_robot.odom);
elseif target_robot.odom(1).robot_id == 2
    target_data_start_point = 3500;
    target_data_end_point = 13000;
elseif target_robot.odom(1).robot_id == 3
    target_data_start_point = 4000;
    target_data_end_point = length(target_robot.odom);
elseif target_robot.odom(1).robot_id == 4
    target_data_start_point = 3500;
    target_data_end_point = length(target_robot.odom);
end

source_robot = data_chopper(source_robot, source_data_start_point, source_data_end_point);
target_robot = data_chopper(target_robot, target_data_start_point, target_data_end_point);

source_data_size = length(source_robot.odom);
target_data_size = length(target_robot.odom);

source_smoothed_poses = load(strcat('robot', string(source_robot_id), '_smoothed_results.mat'));
source_smoothed_poses = getfield(source_smoothed_poses, char(fieldnames(source_smoothed_poses)));

target_smoothed_poses = load(strcat('robot', string(target_robot_id), '_smoothed_results.mat'));
target_smoothed_poses = getfield(target_smoothed_poses, char(fieldnames(target_smoothed_poses)));

target_fiducial_id = target_robot_id - 1;

target_time_pool = zeros(length(target_robot.lmap),1);
for i = 1:length(target_robot.lmap)
    if ~isempty(target_robot.lmap(i).measurement_time)
        target_time_pool(i) = target_robot.lmap(i).measurement_time;
    end
end
target_time_kd_tree = createns(target_time_pool);

encountered_target_pose = [];
expected_target_pose = [];
corresponding_source_smoothed_pose = [];
local_del_fiducial = [];

bb = [];
cc = [];

data_size = min(source_data_size, target_data_size);

for i = 1:data_size
    for j = 1:length(source_robot.fiducial(i).id)
        if source_robot.fiducial(i).id(j) == target_fiducial_id
            source_time = source_robot.fiducial(i).measurement_time;
            closest_target_index = knnsearch(target_time_kd_tree, source_time);
            
%             closest_target_index = find(target_time_pool == closest_target_time);
            closest_target_index_smoothed_results = find(target_smoothed_poses.factor_indices == closest_target_index);
            if isempty(closest_target_index_smoothed_results)
                continue;
            end

            
            source_index_smoothed_results = find(source_smoothed_poses.factor_indices == i);            
            if isempty(source_index_smoothed_results)
                continue;
            end
            current_source_smoothed_pose = source_smoothed_poses.smoothed_results(source_index_smoothed_results, :);
            corresponding_source_smoothed_pose = [corresponding_source_smoothed_pose; current_source_smoothed_pose];

            
            cc = [cc; source_time];
            bb = [bb; target_time_pool(closest_target_index)];
            
            local_del_fiducial_range = source_robot.fiducial(i).range(j);
            local_del_fiducial_bearing = source_robot.fiducial(i).bearing(j);
            local_del_fiducial_pose = [local_del_fiducial_range*cos(local_del_fiducial_bearing), local_del_fiducial_range*sin(local_del_fiducial_bearing)];
            local_del_fiducial = [local_del_fiducial; local_del_fiducial_pose];            
            
            encountered_target_pose = [encountered_target_pose; target_smoothed_poses.smoothed_results(closest_target_index_smoothed_results,:)];            
            expected_target_pose = [expected_target_pose; (current_source_smoothed_pose(1:2)' + (rot(current_source_smoothed_pose(3)) * local_del_fiducial_pose'))'];
        end
    end
end

figure; 
hold on;
for i = 1:size(encountered_target_pose,1)
    plot(encountered_target_pose(i,1), encountered_target_pose(i,2), 'k*')
    plot(expected_target_pose(i,1), expected_target_pose(i,2), 'r*')
    line([encountered_target_pose(i,1) expected_target_pose(i,1)], ...
         [encountered_target_pose(i,2) expected_target_pose(i,2)]);
    drawnow;
end

% legend('self robot pose', 'seen from other robot');

% err = encountered_target_pose(1:end,1:2) - expected_target_pose;
% norm_err;
% for i = 1:length(err)
%     norm_err = 
% end
% figure; 
