function [] = last_day_construction(main_datamat, main_smooth, main_indices, other_datamat, other_smooth, other_indices, online_animation)

% DO NOT PASS LMAP OR POSE INSTEAD OF SMOOTHED_POSES

if nargin == 3
    online_animation = false;
elseif nargin == 4
    online_animation = true;
end

if nargin == 6
    online_animation = false;
end

robot = load(main_datamat);
robot = getfield(robot, char(fieldnames(robot)));

% The length of the dataset
data_size = length(robot.laser);

plot_offset = 1;
if robot.odom(1).robot_id == 1
    plot_offset = 3500; 
elseif robot.odom(1).robot_id == 2
    plot_offset = 3900;
elseif robot.odom(1).robot_id == 3
    plot_offset = 4000;
elseif robot.odom(1).robot_id == 4
    plot_offset = 4100;
end
plot_offset = plot_offset - 1;    

data_end_point = length(main_indices); %6473;

% sine_cache = evalin('base', 'sine_cache');
% cosine_cache = evalin('base', 'cosine_cache');

sine_cache = load('sine_cache.mat');
sine_cache = sine_cache.sine_cache;
cosine_cache = load('cosine_cache.mat');
cosine_cache = cosine_cache.cosine_cache;

k = 0;
scan = [];

figure(1); 
hold on;

if online_animation
    h = plot3(0,0,0);
    g = plot(0,0);

    g(1).LineStyle = 'none';
    g(1).Marker = '.';
    g(1).Color = 'g';
    g(1).MarkerSize = 0.5;

    h(1).LineStyle = 'none';
    h(1).Marker = '.';
    h(1).Color = 'm';
else
    scan = [];    
end

max_scan_range = 15;
scan_project_rate = 10; % One per these many scans.
scan_count = 0;
    
tic
for i = main_indices(1:data_end_point)
    k = k + 1;
    if ~isempty(robot.laser(i+plot_offset).range)
        scan_x = robot.laser(i+plot_offset).range .* cosine_cache;
        scan_y = robot.laser(i+plot_offset).range .* sine_cache;
        
        assert(length(scan_x) == 181);
        assert(length(scan_y) == 181);
        
        filt = find(abs(scan_x) > max_scan_range);
        while ~isempty(filt)
            scan_x(filt(1)) = [];
            scan_y(filt(1)) = [];
            filt = find(abs(scan_x) > max_scan_range);
        end
        filt = find(abs(scan_y) > max_scan_range);
        while ~isempty(filt)
            scan_x(filt(1)) = [];
            scan_y(filt(1)) = [];
            filt = find(abs(scan_y) > max_scan_range);
        end
        
        scan_count = scan_count + 1;
        if online_animation
            scan = [main_smooth(k,1); main_smooth(k,2)] + rot(main_smooth(k,3)) * [scan_x; scan_y];

            if scan_count >= scan_project_rate
                scan_count = 0;
                g(1).XData = [g(1).XData scan(1,:)];
                g(1).YData = [g(1).YData scan(2,:)];
            end
%             h(1).ZData = [h(1).ZData zeros(1,181)];
            h(1).XData = [h(1).XData main_smooth(k,1)];
            h(1).YData = [h(1).YData main_smooth(k,2)];
            h(1).ZData = [h(1).ZData k];
            drawnow;
        else
            if scan_count >= scan_project_rate
                scan_count = 0;
                scan = [scan [main_smooth(k,1); main_smooth(k,2)] + rot(main_smooth(k,3)) * [scan_x; scan_y]];
            end
        end

    end
end
toc

plot(scan(1,:), scan(2,:), 'g.', 'MarkerSize', 0.5);
plot(main_smooth(1:data_end_point,1), main_smooth(1:data_end_point,2), 'm.');

