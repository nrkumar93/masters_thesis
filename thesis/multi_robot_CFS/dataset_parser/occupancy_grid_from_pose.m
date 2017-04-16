function [] = occupancy_grid_from_pose(datamat, smoothed_poses, scan_indices, online_animation)

% DO NOT PASS LMAP OR POSE INSTEAD OF SMOOTHED_POSES

if nargin == 2
    scan_indices = 1:length(smoothed_poses);
    online_animation = false;
elseif nargin == 3
    online_animation = false;
end

robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

% The length of the dataset
data_size = length(robot.laser);

plot_offset = 1;
if robot.odom(1).robot_id == 1
    plot_offset = 3500; 
    start_xy = [0, 0];
elseif robot.odom(1).robot_id == 2
    plot_offset = 3500;
    start_xy = [-1.22, 0.61];    
elseif robot.odom(1).robot_id == 3
    plot_offset = 4000;
    start_xy = [-2.44, 0];
elseif robot.odom(1).robot_id == 4
    plot_offset = 3500;
    start_xy = [-3.66, 0.61];
end
plot_offset = plot_offset - 1;    

data_end_point = length(scan_indices); %6473;

k = 0;
max_range = 15;
map_width = 45;
map_height = 22;
map_cells_per_meter = 20;

smoothed_poses(:,1:2) = smoothed_poses(:,1:2) + [abs(min(smoothed_poses(:,1))) abs(min(smoothed_poses(:,2)))];

map = robotics.OccupancyGrid(map_width, map_height, map_cells_per_meter);

for i = scan_indices(1:data_end_point)
    k = k + 1;
    if ~isempty(robot.laser(i+plot_offset).range)
        scan_ranges = robot.laser(i+plot_offset).range;
        scan_angles = robot.laser(i+plot_offset).bearing;
        
%         filt = find(abs(scan_range) > 15);
%         while ~isempty(filt)
%             scan_range(filt(1)) = [];
%             scan_angles(filt(1)) = [];
%             filt = find(abs(scan_range) > 15);
%         end
        
        insertRay(map, smoothed_poses(k,:), scan_ranges, scan_angles, max_range); 
    end
end

figure;
show(map);
