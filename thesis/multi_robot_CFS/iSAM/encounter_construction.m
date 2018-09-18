function [] = encounter_construction(robot_activation, online_animation)

robot = struct('robot', {}, 'smoothed_poses', {}, 'factor_indices', {});
data_sizes = zeros(1,length(robot_activation));
max_scan_range = 15;
scan_project_rate = 10; % One per these many scans.
x_offset = 32;
y_offset = 20;
scan_count = zeros(1,length(robot_activation));

encounter_data = load('./data/encounter_data.mat');
encounter_data = getfield(encounter_data, char(fieldnames(encounter_data)));

for i=1:length(robot_activation)
    if robot_activation(i) 
        datamat = strcat('./data/dataset_robot', string(i), '.mat');
        robot(i).robot = load(datamat);
        robot(i).robot = getfield(robot(i).robot, char(fieldnames(robot(i).robot)));        
        if i == 1
            data_start_point = 3500; 
            data_end_point = 12500;
        elseif i == 2
            data_start_point = 3900;
            data_end_point = 13000;
        elseif i == 3
            data_start_point = 4000;
            data_end_point = length(robot(i).robot.odom);
        elseif i == 4
            data_start_point = 4100;
            data_end_point = 11000;
        end
        
        robot(i).robot = data_chopper(robot(i).robot, data_start_point, data_end_point);
        
        smoothed_mat = strcat('./data/smoothed_results_', string(i), '.mat');
        robot(i).smoothed_poses = load(smoothed_mat);
        robot(i).smoothed_poses = getfield(robot(i).smoothed_poses, char(fieldnames(robot(i).smoothed_poses)));    
        
        if i == 2
            robot(i).smoothed_poses = robot(i).smoothed_poses + [-x_offset 0 0];
        elseif i == 3
            robot(i).smoothed_poses = robot(i).smoothed_poses + [0 y_offset 0];
        elseif i == 4
            robot(i).smoothed_poses = robot(i).smoothed_poses + [-x_offset y_offset 0];
        end

        factor_indices_mat = strcat('./data/factor_indices_', string(i), '.mat');
        robot(i).factor_indices = load(factor_indices_mat);
        robot(i).factor_indices = getfield(robot(i).factor_indices, char(fieldnames(robot(i).factor_indices)));    
        
        data_sizes(i) = length(robot(i).factor_indices);
    end
end

ordering_time = load('./data/ordering_time.mat');
ordering_time = getfield(ordering_time, char(fieldnames(ordering_time)));
norm_ordering_time.colamd = (ordering_time.colamd - min(ordering_time.colamd))/(max(ordering_time.colamd) - min(ordering_time.colamd));
norm_ordering_time.fusion = (ordering_time.fusion - min(ordering_time.fusion))/(max(ordering_time.fusion) - min(ordering_time.fusion));

fusion_jet = jet(length(norm_ordering_time.fusion));
colamd_jet = jet(length(norm_ordering_time.colamd));

[~, sorted_fusion_order] = sort(norm_ordering_time.fusion);
ordered_fusion_jet = zeros(length(fusion_jet), 3);
k = 1;
for j = sorted_fusion_order
    ordered_fusion_jet(j, :) = fusion_jet(k,:);
    k = k + 1;
end

[~, sorted_colamd_order] = sort(norm_ordering_time.colamd);
ordered_colamd_jet = zeros(length(colamd_jet), 3);
k = 1;
for j = sorted_colamd_order
    ordered_colamd_jet(j, :) = colamd_jet(k,:);
    k = k + 1;
end

construct_data_size = min(data_sizes);

sine_cache = load('sine_cache.mat');
sine_cache = sine_cache.sine_cache;
cosine_cache = load('cosine_cache.mat');
cosine_cache = cosine_cache.cosine_cache;

k = 0;
scan = [];

figure; 
hold on;

if online_animation
    h = plot(0,0,0,0,0,0,0,0);
%     g = plot(0,0,0,0,0,0,0,0);

%     g(1).LineStyle = 'none';
%     g(1).Marker = '.';
%     g(1).Color = [0.7 0.7 1];
%     g(1).MarkerSize = 0.5;
%     
%     g(2).LineStyle = 'none';
%     g(2).Marker = '.';
%     g(2).Color = [0.7 1 0.7];%[0.7 0.7 1];
%     g(2).MarkerSize = 0.5;
%     
%     g(3).LineStyle = 'none';
%     g(3).Marker = '.';
%     g(3).Color = [1 0.7 0.7];
%     g(3).MarkerSize = 0.5;
%     
%     g(4).LineStyle = 'none';
%     g(4).Marker = '.';
%     g(4).Color = [1 0.7 1];
%     g(4).MarkerSize = 0.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    h(1).LineStyle = 'none';
    h(1).Marker = '.';
    h(1).Color = 'b';
    
    h(2).LineStyle = 'none';
    h(2).Marker = '.';
    h(2).Color = 'g';
    
    h(3).LineStyle = 'none';
    h(3).Marker = '.';
    h(3).Color = 'r';
    
    h(4).LineStyle = 'none';
    h(4).Marker = '.';
    h(4).Color = 'm';
    
    e.Color = 'k';
    e.LineWidth = 2;
else
%     h = plot(0,0,0,0,0,0,0,0);
%     h = plot3(zeros(1,data_sizes(1)),0,0,0,0,0,0,0,0,0,0,0);
%     g = plot(0,0,0,0,0,0,0,0);
% 
%     g(1).LineStyle = 'none';
%     g(1).Marker = '.';
%     g(1).Color = [0.7 0.7 1];
%     g(1).MarkerSize = 0.5;
%     
%     g(2).LineStyle = 'none';
%     g(2).Marker = '.';
%     g(2).Color = [0.7 1 0.7];%[0.7 0.7 1];
%     g(2).MarkerSize = 0.5;
%     
%     g(3).LineStyle = 'none';
%     g(3).Marker = '.';
%     g(3).Color = [1 0.7 0.7];
%     g(3).MarkerSize = 0.5;
%     
%     g(4).LineStyle = 'none';
%     g(4).Marker = '.';
%     g(4).Color = [1 0.7 1];
%     g(4).MarkerSize = 0.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
%     h(1).LineStyle = 'none';
%     h(1).Marker = '.';
%     h(1).Color = 'b';
%     
%     h(2).LineStyle = 'none';
%     h(2).Marker = '.';
%     h(2).Color = 'g';
%     
%     h(3).LineStyle = 'none';
%     h(3).Marker = '.';
%     h(3).Color = 'r';
%     
%     h(4).LineStyle = 'none';
%     h(4).Marker = '.';
%     h(4).Color = 'm';
    
    scan_bucket = struct('scan', {});    
    for j = 1:length(robot_activation)
        scan_bucket(j).scan = [];
    end
    
    encounter_bucket = struct('source', struct('x', [], 'y', [], 'z', []), 'target', struct('x', [], 'y', [], 'z', []));
    encounter_bucket = [encounter_bucket encounter_bucket encounter_bucket encounter_bucket];
%     encounter_bucket.source = [encounter_bucket.source encounter_bucket.source encounter_bucket.source encounter_bucket.source];
%     encounter_bucket.target = [encounter_bucket.target encounter_bucket.target encounter_bucket.target encounter_bucket.target];
end

i = ones(1,length(robot_activation));
ec_marker = ones(1,length(robot_activation));


while 1
    for j = 1:length(robot_activation)
        while i(j) <= data_sizes(j)
            if robot_activation(j)
                if ~isempty(robot(j).robot.laser(robot(j).factor_indices(i(j))).range)
%                     scan_x = robot(j).robot.laser(robot(j).factor_indices(i(j))).range .* cosine_cache;
%                     scan_y = robot(j).robot.laser(robot(j).factor_indices(i(j))).range .* sine_cache;

%                     filt = find(abs(scan_x) > max_scan_range);
%                     while ~isempty(filt)
%                         scan_x(filt(1)) = [];
%                         scan_y(filt(1)) = [];
%                         filt = find(abs(scan_x) > max_scan_range);
%                     end
%                     filt = find(abs(scan_y) > max_scan_range);
%                     while ~isempty(filt)
%                         scan_x(filt(1)) = [];
%                         scan_y(filt(1)) = [];
%                         filt = find(abs(scan_y) > max_scan_range);
%                     end

%                     scan_count(j) = scan_count(j) + 1;
                    if online_animation
%                         scan = [robot(j).smoothed_poses(i(j),1); ...
%                                 robot(j).smoothed_poses(i(j),2)] + rot(robot(j).smoothed_poses(i(j),3)) * [scan_x; scan_y];

%                         if scan_count(j) >= scan_project_rate
%                             scan_count(j) = 0;
%                             g(j).XData = [g(j).XData scan(1,:)];
%                             g(j).YData = [g(j).YData scan(2,:)];
%                         end
                        h(j).XData = [h(j).XData robot(j).smoothed_poses(i(j),1)];
                        h(j).YData = [h(j).YData robot(j).smoothed_poses(i(j),2)];
                        
                        if encounter_data(j).encounter(ec_marker(j)).source_factor_index == robot(j).factor_indices(i(j))
                            kutti = encounter_data(j).encounter(ec_marker(j));
                            for f = 1:length(kutti.fiducial)
%                                 if robot(kutti.fiducial(f)).factor_indices(i(kutti.fiducial(f))) > kutti.target_factor_index(f)
                                    target_pose = robot(kutti.fiducial(f)).smoothed_poses(i(kutti.fiducial(f)),:);
                                    source_pose = robot(j).smoothed_poses(i(j),:);
                                    plot([target_pose(1) source_pose(1)], [target_pose(2) source_pose(2)], 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
%                                 end
                            end
                            ec_marker(j) = ec_marker(j) + 1;
                        end
                        
                        drawnow;                    
                    else
%                         if scan_count(j) >= scan_project_rate
%                             scan_count(j) = 0;
%                             scan_bucket(j).scan = [scan_bucket(j).scan ...
%                                                   [robot(j).smoothed_poses(i(j),1); ...
%                                                    robot(j).smoothed_poses(i(j),2)] + rot(robot(j).smoothed_poses(i(j),3)) * [scan_x; scan_y]];
%                         end

%                         kutti = encounter_data(j).encounter;
%                         for l = 1:length(kutti)
%                             for f = 1:length(kutti(l).fiducial)
%                                 source_pose = robot(j).smoothed_poses(kutti(l).source_factor_index,:);
%                                 target_pose = robot(kutti(l).fiducial(f)).smoothed_poses(kutti(l).target_factor_index,:);
% 
%                                 encounter_bucket(j).source.x = [encounter_bucket(j).source.x source_pose(1)];
%                                 encounter_bucket(j).source.y = [encounter_bucket(j).source.y source_pose(2)];
% 
%                                 encounter_bucket(j).target.x = [encounter_bucket(j).target.x target_pose(1)];
%                                 encounter_bucket(j).target.y = [encounter_bucket(j).target.y target_pose(2)];
%                             end
%                         end
                        
%                         if encounter_data(j).encounter(ec_marker(j)).source_factor_index == robot(j).factor_indices(i(j))
%                             kutti = encounter_data(j).encounter(ec_marker(j));
%                             for f = 1:length(kutti.fiducial)
% %                                 if robot(kutti.fiducial(f)).factor_indices(i(kutti.fiducial(f))) > kutti.target_factor_index(f)
%                                     target_pose = robot(kutti.fiducial(f)).smoothed_poses(i(kutti.fiducial(f)),:);
%                                     source_pose = robot(j).smoothed_poses(i(j),:);
%                                     
%                                     encounter_bucket(j).source.x = [encounter_bucket(j).source.x source_pose(1)];
%                                     encounter_bucket(j).source.y = [encounter_bucket(j).source.y source_pose(2)];
%                                     encounter_bucket(j).source.z = [encounter_bucket(j).source.z i(j)];
% 
%                                     encounter_bucket(j).target.x = [encounter_bucket(j).target.x target_pose(1)];
%                                     encounter_bucket(j).target.y = [encounter_bucket(j).target.y target_pose(2)];
%                                     encounter_bucket(j).target.z = [encounter_bucket(j).target.z i(kutti.fiducial(f))];
% %                                 end
%                             end
%                             ec_marker(j) = ec_marker(j) + 1;
%                         end

                    end
                end
                i(j) = i(j) + 1;
            end
            break;
        end
    end
    if isequal(i,data_sizes+1)
        break;
    end
end

if ~online_animation
    for j = 1:length(robot_activation)
        kutti = encounter_data(j).encounter;
        for l = 1:length(kutti)
            for f = 1:length(kutti(l).fiducial)
                if length(robot(kutti(l).fiducial(f)).smoothed_poses) > kutti(l).target_factor_index
                    source_pose = robot(j).smoothed_poses(kutti(l).source_factor_index,:);
                    target_pose = robot(kutti(l).fiducial(f)).smoothed_poses(kutti(l).target_factor_index(f),:);
                    
                    encounter_bucket(j).source.x = [encounter_bucket(j).source.x source_pose(1)];
                    encounter_bucket(j).source.y = [encounter_bucket(j).source.y source_pose(2)];

                    encounter_bucket(j).target.x = [encounter_bucket(j).target.x target_pose(1)];
                    encounter_bucket(j).target.y = [encounter_bucket(j).target.y target_pose(2)];
       
                end
            end
        end
    end
end



asc_order = [3 2 4 1];
if ~online_animation
    color = struct('red', []);
    color = [color color color color];
    jet_color = struct('fusion', [], 'colamd', []);
    jet_color = [jet_color jet_color jet_color jet_color];
    for j = asc_order
        if robot_activation(j)
            splitsize = floor(length(norm_ordering_time.fusion)/length(encounter_bucket(j).source.x));
            tbr = [];
            for k = 1:length(encounter_bucket(j).source.x)
                pointer = (k-1)*splitsize + 1;
                color(j).red = [color(j).red norm_ordering_time.fusion(pointer)];
                jet_color(j).fusion = [jet_color(j).fusion; ordered_fusion_jet(pointer, :)];
                jet_color(j).colamd = [jet_color(j).colamd; ordered_colamd_jet(pointer, :)];
                
                tbr = [tbr pointer];
            end
            norm_ordering_time.fusion(tbr) = [];
        end
    end
end



if ~online_animation
    for j = 1:length(robot_activation)
        if robot_activation(j)
%             g(j).XData = [g(j).XData scan_bucket(j).scan(1,:)];
%             g(j).YData = [g(j).YData scan_bucket(j).scan(2,:)]; 

            jumper = length(robot_activation) -j + 1; 
            for k = 1:length(encounter_bucket(j).source.x)
                source_pose = [encounter_bucket(j).source.x(k) encounter_bucket(j).source.y(k)];
                target_pose = [encounter_bucket(j).target.x(k) encounter_bucket(j).target.y(k)];
                
%                 pointer = (k-1)*jumper + 1;
%                 red = norm_ordering_time.fusion(pointer);
%                 green =  1 - red;
%                 norm_ordering_time.fusion(pointer) = [];
                plot([target_pose(1) source_pose(1)], [target_pose(2) source_pose(2)], 'Color', jet_color(j).colamd(k,:), 'LineWidth', 0.2);
            end
        end
    end
end


if ~online_animation
    h = plot(0,0,0,0,0,0,0,0);
    
    h(1).LineStyle = 'none';
    h(1).Marker = '.';
%     h(1).Color = 'b';
    h(1).Color = 'k';
    
    h(2).LineStyle = 'none';
    h(2).Marker = '.';
%     h(2).Color = 'g';
    h(2).Color = 'k';
    
    h(3).LineStyle = 'none';
    h(3).Marker = '.';
%     h(3).Color = 'r';
    h(3).Color = 'k';
    
    h(4).LineStyle = 'none';
    h(4).Marker = '.';
%     h(4).Color = 'm';
    h(4).Color = 'k';

    for j = 1:length(robot_activation)
        if robot_activation(j)
            for k = 1:data_sizes(j)
                h(j).XData = [h(j).XData robot(j).smoothed_poses(k,1)];
                h(j).YData = [h(j).YData robot(j).smoothed_poses(k,2)];
            end            
        end
    end
%     figure; hold on;
%     for j = 1:length(robot_activation)
%         if robot_activation(j)
%             plot(robot(j).smoothed_poses(:,1), robot(j).smoothed_poses(:,2), '.');
%         end
%     end
    
end
