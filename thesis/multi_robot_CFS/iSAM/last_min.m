function [] = last_min(datamat, smoothed_poses, sm_1, sm_2, sm_4,  scan_indices, online_animation)

% DO NOT PASS LMAP OR POSE INSTEAD OF SMOOTHED_POSES

if nargin == 6
%     scan_indices = 1:length(smoothed_poses);
    online_animation = false;
% elseif nargin == 3
%     online_animation = false;
end

robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

% The length of the dataset
data_size = length(robot.laser);

plot_offset = 1;
if robot.odom(1).robot_id == 1
    plot_offset = 3500; 
elseif robot.odom(1).robot_id == 2
    plot_offset = 3500;
elseif robot.odom(1).robot_id == 3
    plot_offset = 4000;
elseif robot.odom(1).robot_id == 4
    plot_offset = 3500;
end
plot_offset = plot_offset - 1;    

data_end_point = length(scan_indices); %6473;

% sine_cache = evalin('base', 'sine_cache');
% cosine_cache = evalin('base', 'cosine_cache');

sine_cache = load('sine_cache.mat');
sine_cache = sine_cache.sine_cache;
cosine_cache = load('cosine_cache.mat');
cosine_cache = cosine_cache.cosine_cache;

k = 0;
scan = [];

figure; 
hold on;

if online_animation
    h = plot(0,0,0,0,0,0,0,0,0,0);

    h(1).LineStyle = 'none';
    h(1).Marker = '.';
    h(1).Color = 'k';
    h(1).MarkerSize = 4; 
    h(2).LineStyle = 'none';
    h(2).Marker = '.';
    h(2).Color = 'r';
    h(3).LineStyle = 'none';
    h(3).Marker = '.';
    h(3).Color = 'b';
    h(4).LineStyle = 'none';
    h(4).Marker = '.';
    h(4).Color = 'g';
    h(5).LineStyle = 'none';
    h(5).Marker = '.';
    h(5).Color = 'y';
    
else
    scan = [];    
end
    
tic
for i = scan_indices(1:data_end_point)
    k = k + 1;
    if ~isempty(robot.laser(i+plot_offset).range)
        scan_x = robot.laser(i+plot_offset).range .* cosine_cache;
        scan_y = robot.laser(i+plot_offset).range .* sine_cache;
        filt = find(abs(scan_x) > 15);
        while ~isempty(filt)
            scan_x(filt(1)) = [];
            scan_y(filt(1)) = [];
            filt = find(abs(scan_x) > 15);
        end
        filt = find(abs(scan_y) > 15);
        while ~isempty(filt)
            scan_x(filt(1)) = [];
            scan_y(filt(1)) = [];
            filt = find(abs(scan_y) > 15);
        end
        
        if online_animation
            scan = [smoothed_poses(k,1); smoothed_poses(k,2)] + rot(smoothed_poses(k,3)) * [scan_x; scan_y];

            h(1).XData = [h(1).XData scan(1,:)];
            h(1).YData = [h(1).YData scan(2,:)];
            h(2).XData = [h(2).XData smoothed_poses(k,1)];
            h(2).YData = [h(2).YData smoothed_poses(k,2)];
            h(3).XData = [h(3).XData sm_1(k,1)];
            h(3).YData = [h(3).YData sm_1(k,2)];
            h(4).XData = [h(4).XData sm_2(k,1)];
            h(4).YData = [h(4).YData sm_2(k,2)];
            h(5).XData = [h(5).XData sm_4(k,1)];
            h(5).YData = [h(5).YData sm_4(k,2)];
            drawnow;
        else
            scan = [scan [smoothed_poses(k,1); smoothed_poses(k,2)] + rot(smoothed_poses(k,3)) * [scan_x; scan_y]];
        end

    end
end
toc

plot(scan(1,:), scan(2,:), 'k.');
plot(smoothed_poses(1:data_end_point,1), smoothed_poses(1:data_end_point,2), 'r.');

