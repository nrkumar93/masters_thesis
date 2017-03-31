function [lmap] = lmap_parser(raw_lmap)

lmap = struct('server_time', {}, 'robot_id', {}, 'measurement_time', {}, 'pose', {});

if ~iscell(raw_lmap)
    if raw_lmap == false
        lmap = struct('server_time', [], 'robot_id', [], 'measurement_time', [], 'pose', []);
        return;
    end
end

lmap(1).server_time = str2double(raw_lmap(1));

if strcmp(raw_lmap(2), 'intel0')
    lmap(1).robot_id = 1;
elseif strcmp(raw_lmap(2), 'intel1')
    lmap(1).robot_id = 2;
elseif strcmp(raw_lmap(2), 'intel2')
    lmap(1).robot_id = 3;
elseif strcmp(raw_lmap(2), 'intel3')
    lmap(1).robot_id = 4;
end

lmap(1).measurement_time = str2double(raw_lmap(6));

lmap(1).pose = cellfun(@str2num, raw_lmap(7:9));
