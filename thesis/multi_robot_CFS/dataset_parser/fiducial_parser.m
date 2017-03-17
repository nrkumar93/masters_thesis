function [fiducial] = fiducial_parser(raw_fiducial)

fiducial = struct('server_time', {}, 'robot_id', {}, 'measurement_time', {}, ...
                  'id', {}, 'range', {}, 'bearing', {}, 'orient', {});

if ~iscell(raw_fiducial)
    if raw_fiducial == false
        fiducial = struct('server_time', [], 'robot_id', [], 'measurement_time', [], ...
                          'id', [], 'range', [], 'bearing', [], 'orient', []);
        return;
    end
end

fiducial(1).server_time = str2double(raw_fiducial(1));

if strcmp(raw_fiducial(2), 'intel0')
    fiducial(1).robot_id = 1;
elseif strcmp(raw_fiducial(2), 'intel1')
    fiducial(1).robot_id = 2;
elseif strcmp(raw_fiducial(2), 'intel2')
    fiducial(1).robot_id = 3;
elseif strcmp(raw_fiducial(2), 'intel3')
    fiducial(1).robot_id = 4;
end

fiducial(1).measurement_time = str2double(raw_fiducial(6));

fiducial(1).id = cellfun(@str2num, raw_fiducial(7:4:length(raw_fiducial)-1));

fiducial(1).range = cellfun(@str2num, raw_fiducial(8:4:length(raw_fiducial)-1));

fiducial(1).bearing = cellfun(@str2num, raw_fiducial(9:4:length(raw_fiducial)-1));

fiducial(1).orient = cellfun(@str2num, raw_fiducial(10:4:length(raw_fiducial)-1));