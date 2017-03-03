function [odom] = odom_parser(raw_odom)

odom = struct('server_time', {}, 'robot_id', {}, 'measurement_time', {}, 'pose', {}, 'velocity', {});

odom(1).server_time = str2double(raw_odom(1));

if strcmp(raw_odom(2), 'intel0')
    odom(1).robot_id = 1;
elseif strcmp(raw_odom(2), 'intel1')
    odom(1).robot_id = 2;
elseif strcmp(raw_odom(2), 'intel2')
    odom(1).robot_id = 3;
elseif strcmp(raw_odom(2), 'intel3')
    odom(1).robot_id = 4;
end

odom(1).measurement_time = str2double(raw_odom(6));

odom(1).pose = cellfun(@str2num, raw_odom(7:9));

odom(1).velocity = cellfun(@str2num, raw_odom(10:12));
