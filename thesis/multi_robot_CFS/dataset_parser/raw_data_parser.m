function dataset = raw_data_parser(data_stream)

dataset = struct('odom', {}, 'laser', {}, 'fiducial', {}, 'lmap', {}, 'sync', {});

laser_i = 1;
odom_i = 1;
fiducial_i = 1;
lmap_i = 1;
sync_i = 1;

while ~feof(data_stream)
    unit_measurement = strsplit(fgets(data_stream));
    if strcmp(unit_measurement(4),'position') 
        dataset(1).odom(odom_i) = odom_parser(unit_measurement);
        if odom_i - laser_i == 1
            dataset(1).laser(laser_i) = laser_parser(false);
            laser_i = laser_i + 1;
        end
        if odom_i - fiducial_i == 1
            dataset(1).fiducial(fiducial_i) = fiducial_parser(false);
            fiducial_i = fiducial_i + 1;
        end
        if odom_i - lmap_i == 1
            dataset(1).lmap(lmap_i) = lmap_parser(false);
            lmap_i = lmap_i + 1;        
        end
        if odom_i - sync_i == 1
            dataset(1).sync(sync_i) = struct('sync', []);
            sync_i = sync_i + 1;        
        end        
        odom_i = odom_i + 1;
        continue;
    elseif strcmp(unit_measurement(4),'laser')
        dataset(1).laser(laser_i) = laser_parser(unit_measurement);
        laser_i = laser_i + 1;
        continue;
    elseif strcmp(unit_measurement(4), 'fiducial')
        dataset(1).fiducial(fiducial_i) = fiducial_parser(unit_measurement);
        fiducial_i = fiducial_i + 1;
        continue;
    elseif strcmp(unit_measurement(4), 'lmap')
        dataset(1).lmap(lmap_i) = lmap_parser(unit_measurement);
        lmap_i = lmap_i + 1;
        continue;
    elseif strcmp(unit_measurement(4), 'sync')
        dataset(1).sync(sync_i) = struct('sync', str2double(unit_measurement(6)));
        sync_i = sync_i + 1;
        continue;
    end
end
