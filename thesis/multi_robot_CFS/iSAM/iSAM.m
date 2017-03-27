clear;

import gtsam.*

%% Load the saved robot specific dataset.
datamat = './data/dataset_robot3.mat';
robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

data_start_point = 4000;
data_end_point = length(robot.odom);

%% Examining with a part of the entire dataset.
robot = data_chopper(robot, data_start_point, data_end_point);

load('sine_cache.mat');
load('cosine_cache.mat');

%% The length of the dataset
data_size = length(robot.odom);
start_offset = 0;
end_offset = 0;

%% Loading the time delta.
meas_t = [];
meas_t = [meas_t, robot.odom.measurement_time];
del_t = diff(meas_t);
avg_del_t = mean(del_t);

%% Loading all the odometry model values from the data.
all_poses = [];
all_poses = [all_poses, robot.odom.pose];
pose_x = all_poses(1:3:end);
pose_y = all_poses(2:3:end);
pose_theta = all_poses(3:3:end);

%% Loading all the velocity command values from the data.
vels = [];
vels = [vels, robot.odom.velocity];
lin_vel = vels(1:3:end);
ang_vel = vels(3:3:end);

%% The JSON params and other params.
params = loadjson('parameters_iSAM.json');
batchInitialization=true;
batch_update_size = 200; % minimum number of range measurements to process initially
isam_update_rate = 10;

%% Scan Matching Mode
% 1. icpmatlab
% 2. csm_icp_matlab
% 3. csm_icp_c++
scan_matching_mode = params{1}.scan_matching.MODE;
if params{1}.scan_matching.KEYFRAME
    key_frame_rate = params{1}.scan_matching.KEYFRAME_RATE;
else
    key_frame_rate = 1;
end

%% Type of constraints to be added
scan_matching_flag = params{2}.constraints.SCAN_MATCHING_CONSTRAINTS;
odometry_flag = params{2}.constraints.ODOMETRY_CONSTRAINTS;
velocity_model_flag = params{2}.constraints.VELOCITY_MODEL_CONSTRAINTS;

%% Parsing the noise models from JSON
scan_matching_covariance = params{3}.covariance.SCAN_MATCHING;
odometry_covariance = params{3}.covariance.ODOMETRY;
velocity_model_covariance = params{3}.covariance.VELOCITY_MODEL;
fiducial_covariance = params{3}.covariance.FIDUCIAL;

%% Time scaled covariance
odometry_covariance_per_time_ratio = odometry_covariance/(avg_del_t*avg_del_t);
velocity_model_covariance_per_time_ratio = velocity_model_covariance/(avg_del_t*avg_del_t);

%% Initialize iSAM and Non linear Factor Graph
isam = ISAM2;
graph = NonlinearFactorGraph;
result = Values;

key =1;
count = 0;

%% Initial Estimates

initial = Values;
init_x = 0.0; init_y = 0.0; init_theta = 0.0;
initial.insert(key, Pose2(init_x, init_y, init_theta));

%% Adding prior with arbitrary prior covariance.
priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
graph.add(PriorFactorPose2(key, Pose2(0, 0, 0), priorNoise));


if scan_matching_flag && odometry_flag && velocity_model_flag
    for i=1:data_size-end_offset-1
%% ODOMETRY DEAD RECKONING CONSTRAINTS
%         Calculating odometry using dead reckoning
        [o_del_x, o_del_y, o_del_theta] = odometry_difference(pose_x(i), pose_y(i), pose_theta(i), ...
                                                              pose_x(i+1), pose_y(i+1), pose_theta(i+1));

% Delta time scaled Odometry Dead Reckoning covariance. 
        odom_noise = noiseModel.Diagonal.Sigmas([odometry_covariance_per_time_ratio(1); ...
                                                 odometry_covariance_per_time_ratio(5); ...
                                                 odometry_covariance_per_time_ratio(9)] * (del_t(i)^2));
%         Adding odometry contraints to graph
        graph.add(BetweenFactorPose2(key, key+1, Pose2(o_del_x, o_del_y, o_del_theta), odom_noise));

%% ODOMETRY VELOCITY MODEL CONSTRAINTS       
% %          Calculating delta pose using velocities
%         [v_del_x, v_del_y, v_del_theta] = velocity_motion_model(lin_vel(i), ang_vel(i), pose_theta(i), del_t(i));        
% 
% % Velocity model covariance.
%         velocity_model_noise = noiseModel.Diagonal.Sigmas([velocity_model_covariance_per_time_ratio(1); ...
%                                                            velocity_model_covariance_per_time_ratio(5); ...
%                                                            velocity_model_covariance_per_time_ratio(9)] * (del_t(i)^2));
%         
% % %          Adding delpa pose from velocity constraints 
%         graph.add(BetweenFactorPose2(key, key+1, Pose2(v_del_x, v_del_y, v_del_theta), velocity_model_noise));
%         
%% LASER SCAN MATCHING CONSTRAINTS
%         Calling Scan Matcher and filtering for some modes
    
        if key_frame_rate == 1
            frame_id = i;
        elseif mod(i, key_frame_rate) == 1
            frame_id = i;
        end

        if scan_matching_mode == 1 %  icpmatlab
            [scan_match_R, scan_match_T] = scan_matcher(robot.laser(frame_id).range, robot.laser(i+1).range);
        elseif scan_matching_mode == 2 % csm_icp_matlab
            [scan_match_R, scan_match_T] = csm_scan_matcher(robot.laser(frame_id).range, robot.laser(i+1).range);
        elseif scan_matching_mode == 3 % csm_icp_c++
            [csm_init_x, csm_init_y, csm_init_theta] = ...
                odometry_difference(pose_x(frame_id), pose_y(frame_id), pose_theta(frame_id), ...
                                    pose_x(i+1), pose_y(i+1), pose_theta(i+1));
            
            [scan_match_R, scan_match_T, scan_matching_noise, scan_matching_theta] = ...
                fast_csm_scan_matcher(robot.laser(frame_id).measurement_time, robot.laser(frame_id).range, ...
                robot.laser(i+1).measurement_time, robot.laser(i+1).range, [csm_init_x, csm_init_y, csm_init_theta]);
            
            if ~isempty(scan_match_R) && ~isempty(scan_match_T)
                scan_matching_noise = noiseModel.Gaussian.Covariance(reshape(scan_matching_noise, [3 3]));
            end
        end

%         Throw away bizzare scan match results
        if ~isempty(scan_match_R) && ~isempty(scan_match_T)
            [odom_frame_x, odom_frame_y, ~] = odometry_difference(pose_x(frame_id), pose_y(frame_id), ...
                pose_theta(frame_id), pose_x(i+1), pose_y(i+1), pose_theta(i+1));
            disparity = abs(norm([odom_frame_x, odom_frame_y]) - norm(scan_match_T));
            
            if disparity < (0.01 * key_frame_rate) || norm(scan_match_T) < (key_frame_rate * 0.1)
                graph.add(BetweenFactorPose2(frame_id, key+1, ...
                    Pose2(scan_match_T(1), scan_match_T(2), scan_matching_theta), scan_matching_noise)); 
            end
        end

%% FIDUCIAL CONSTRAINTS 
            
        if ~isempty(robot.fiducial(i).id)
            for j = 1:length(robot.fiducial(i).id)
                if robot.fiducial(i).id(j) > 3 %|| robot.fiducial(i).id(j) == -1
                    [fid_del_x, fid_del_y, fid_del_theta] = ...
                        fiducial_processor(robot.fiducial(i).range(j), robot.fiducial(i).bearing(j));

                    del_fid = Pose2(fid_del_x, fid_del_y, fid_del_theta);
                    landmark_key = symbol('l', robot.fiducial(i).id(j));
                    
                    % Fiducial covariance. Very low to assert almost zero error.
                    fiducial_noise = noiseModel.Diagonal.Sigmas([fiducial_covariance(1); ...
                                                                 fiducial_covariance(5); ...
                                                                 fiducial_covariance(9)]);
                    
                    graph.add(BetweenFactorPose2(key, landmark_key, del_fid, fiducial_noise));
                    
                    if ~initial.exists(landmark_key) && ~result.exists(landmark_key)
                        fid_init_point2 = (([init_x; init_y]) + rot(init_theta) * [fid_del_x; fid_del_y])';
                        initial.insert(landmark_key, Pose2(fid_init_point2(1), fid_init_point2(2), 0));
                    elseif result.exists(landmark_key) && (i > batch_update_size)
                        estimated_global_pose2 = Pose2(init_x, init_y, init_theta);
                        graph.add(BetweenFactorPose2(key, landmark_key, ...
                            estimated_global_pose2.between(result.at(landmark_key)), noiseModel.Diagonal.Sigmas([0.00001; 0.00001; 0.00001])));                            
                    end                
                end
            end
        end
        
%%
        key = key + 1;
        count = count + 1;
        
%% INITIAL GUESS
%         Adding initial guess.
%         [v_del_x, v_del_y, v_del_theta] = velocity_motion_model(lin_vel(i), ang_vel(i), pose_theta(i), del_t(i));
        [v_del_x, v_del_y, v_del_theta] = odometry_difference(pose_x(i), pose_y(i), pose_theta(i), ...
                                                              pose_x(i+1), pose_y(i+1), pose_theta(i+1));
        init_x = init_x + v_del_x;
        init_y = init_y + v_del_y;
        init_theta = init_theta + v_del_theta;
        initial.insert(key, Pose2(init_x, init_y, init_theta));

        if i > batch_update_size && count > isam_update_rate
            if batchInitialization
                batchOptimizer = LevenbergMarquardtOptimizer(graph, initial);
                initial = batchOptimizer.optimize();
                batchInitialization = false; % only once                
            end
            isam.update(graph, initial);
            result = isam.calculateEstimate();
            init_x = result.at(key).x;
            init_y = result.at(key).y;
            init_theta = result.at(key).theta;
            graph = NonlinearFactorGraph;
            initial = Values;
            count = 0;            
        end        
    end
end

% result
isam.update(graph, initial);
result = isam.calculateEstimate();
XYT = utilities.extractPose2(result);

% figure; hold on;
% plot(XYT(:,1),XYT(:,2),'k-');

% Retrieving the smoothed data.
smoothed_results = zeros(key, 3);
for k = 1:key
    smoothed_results(k,:) = [result.at(k).x, result.at(k).y, result.at(k).theta];
end

% Plotting

figure; hold on;
plot(smoothed_results(:,1), smoothed_results(:,2));

