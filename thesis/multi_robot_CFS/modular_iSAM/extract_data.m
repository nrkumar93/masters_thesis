function data = extract_data(robot_activation_mask, robot_data_end)

for i = 1:length(robot_activation_mask)
    if robot_activation_mask(i) ~= 0
        datamat = strcat('./data/dataset_robot', string(i), '.mat');
        robot = load(datamat);
        robot = getfield(robot, char(fieldnames(robot)));        
        if i == 1
            data_start_point = 3500; 
        elseif i == 2
            data_start_point = 3900;
        elseif i == 3
            data_start_point = 4000;
        elseif i == 4
            data_start_point = 4100;
        end
        
        % Examining with a part of the entire dataset.
        if robot_data_end(i) == inf
            data_end_point = length(robot.odom);
        else
            data_end_point = robot_data_end(i);
        end
        robot = data_chopper(robot, data_start_point, data_end_point);
        
        % The length of the dataset
        data(i).data_size = length(robot.odom);

        % Loading the time delta.
        meas_t = [];
        meas_t = [meas_t, robot.odom.measurement_time];
        data(i).del_t = diff(meas_t);
        data(i).avg_del_t = mean(data(i).del_t);
        data(i).odom_meas_t = meas_t;
        
        % Loading all the odometry model values from the data.
        all_poses = [];
        all_poses = [all_poses, robot.odom.pose];
        data(i).odom = [all_poses(1:3:end)' all_poses(2:3:end)' all_poses(3:3:end)'];
        
        % Loading all the velocity command values from the data.
        vels = [];
        vels = [vels, robot.odom.velocity];
        data(i).velocity = [vels(1:3:end)' vels(3:3:end)'];
        
        % Loading lmap from the data. DO NOT USE THESE. INDICES CRUNCHED.
        data(i).lmap = robot.lmap;

        % Loading the LIDAR ranges. Not loading the scan angles. Just
        % using the knowledge of the problem. 
        data(i).laser = robot.laser;
        
        % Load loop closures
        data(i).closures = robot.lclosures;
        
        % Load fiducials
        data(i).fiducials = robot.fiducial;        
    end
end
