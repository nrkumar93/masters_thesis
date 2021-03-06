function [laser] = laser_parser(raw_laser)

laser = struct('server_time', {}, 'robot_id', {}, 'measurement_time', {}, 'range', {}, 'bearing', {}, 'intensity', {});

if ~iscell(raw_laser)
    if raw_laser == false
        laser = struct('server_time', [], 'robot_id', [], 'measurement_time', [], 'range', [], 'bearing', [], 'intensity', []);
        return;
    end
end

laser(1).server_time = str2double(raw_laser(1));

if strcmp(raw_laser(2), 'intel0')
    laser(1).robot_id = 1;
elseif strcmp(raw_laser(2), 'intel1')
    laser(1).robot_id = 2;
elseif strcmp(raw_laser(2), 'intel2')
    laser(1).robot_id = 3;
elseif strcmp(raw_laser(2), 'intel3')
    laser(1).robot_id = 4;
end

laser(1).measurement_time = str2double(raw_laser(6));

laser(1).range = cellfun(@str2num, raw_laser(7:3:length(raw_laser)-1));

laser(1).bearing = cellfun(@str2num, raw_laser(8:3:length(raw_laser)-1));

laser(1).intensity = cellfun(@str2num, raw_laser(9:3:length(raw_laser)-1));