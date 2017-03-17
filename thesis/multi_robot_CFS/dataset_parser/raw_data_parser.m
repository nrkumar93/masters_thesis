function dataset = raw_data_parser(data_stream)

dataset = struct('odom', {}, 'laser', {}, 'fiducial', {});

laser_i = 1;
odom_i = 1;
fiducial_i = 1;

while ~feof(data_stream)
    unit_measurement = strsplit(fgets(data_stream));
    if strcmp(unit_measurement(4),'position') 
        dataset(1).odom(odom_i) = odom_parser(unit_measurement);
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
    end
    if odom_i - laser_i == 1
        dataset(1).laser(laser_i) = laser_parser(false);
        laser_i = laser_i + 1;
    elseif odom_i - fiducial_i == 1
        dataset(1).fiducial(fiducial_i) = fiducial_parser(false);
        fiducial_i = fiducial_i + 1;
    end
end