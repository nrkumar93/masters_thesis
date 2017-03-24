import gtsam.*

% Load the saved robot specific dataset.
datamat = './data/dataset_robot3.mat';
robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

load('sine_cache.mat');
load('cosine_cache.mat');

% The length of the dataset
data_size = length(robot.odom);
start_offset = 0;
end_offset = 0;

% Loading the time delta.
meas_t = [];
meas_t = [meas_t, robot.odom.measurement_time];
del_t = diff(meas_t);

% Loading all the odometry model values from the data.
all_poses = [];
all_poses = [all_poses, robot.odom.pose];
pose_x = all_poses(1:3:end);
pose_y = all_poses(2:3:end);
pose_theta = all_poses(3:3:end);

% Loading all the velocity command values from the data.
vels = [];
vels = [vels, robot.odom.velocity];
lin_vel = vels(1:3:end);
ang_vel = vels(3:3:end);

% The JSON params 
params = loadjson('parameters.json');

% Scan Matching Mode
% 1. icpmatlab
% 2. csm_icp_matlab
% 3. csm_icp_c++
scan_matching_mode = params{1}.scan_matching.MODE;

scan_matching_covariance = params{3}.covariance.SCAN_MATCHING;
odometry_covariance = params{3}.covariance.ODOMETRY;
velocity_model_covariance = params{3}.covariance.VELOCITY_MODEL;

scan_matching_flag = params{2}.constraints.SCAN_MATCHING_CONSTRAINTS;
odometry_flag = params{2}.constraints.ODOMETRY_CONSTRAINTS;
velocity_model_flag = params{2}.constraints.VELOCITY_MODEL_CONSTRAINTS;

graph = NonlinearFactorGraph;
initial = Values;
init_x = 0.0; init_y = 0.0; init_theta = 0.0;
key =1;

initial.insert(key, Pose2(init_x, init_y, init_theta));

%% Adding prior with arbitrary prior covariance.
priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
graph.add(PriorFactorPose2(key, Pose2(0, 0, 0), priorNoise));

%% COVARIANCE OF VARIOUS MODELS

% LIDAR scan matching covariance. For mode 3, the covariance is provided
% by the scan matching algorithm.
scan_matching_noise = noiseModel.Diagonal.Sigmas([scan_matching_covariance(1); ...
                                                  scan_matching_covariance(5); ...
                                                  scan_matching_covariance(9);]);
% Odometry Dead Reckoning covariance. 
% TODO: Covariance proportional to delta t.
odom_noise = noiseModel.Diagonal.Sigmas([odometry_covariance(1); ...
                                         odometry_covariance(5); ...
                                         odometry_covariance(9);]);
% Velocity model covariance.
% TODO: Covariance proportional to delta t.
velocity_model_noise = noiseModel.Diagonal.Sigmas([velocity_model_covariance(1); ...
                                                  velocity_model_covariance(5); ...
                                                  velocity_model_covariance(9);]);

if scan_matching_flag && odometry_flag && velocity_model_flag
    for i=1:data_size-end_offset-1
%% ODOMETRY DEAD RECKONING CONSTRAINTS
%         Calculating odometry using dead reckoning
        [o_del_x, o_del_y, o_del_theta] = odometry_difference(pose_x(i), pose_y(i), pose_theta(i), ...
                                                              pose_x(i+1), pose_y(i+1), pose_theta(i+1));
%         Adding odometry contraints to graph
        graph.add(BetweenFactorPose2(key, key+1, Pose2(o_del_x, o_del_y, o_del_theta), odom_noise));

%% ODOMETRY VELOCITY MODEL CONSTRAINTS       
%          Calculating delta pose using velocities
%         [v_del_x, v_del_y, v_del_theta] = velocity_motion_model(lin_vel(i), ang_vel(i), pose_theta(i), del_t(i));        
%          Adding delpa pose from velocity constraints 
%         graph.add(BetweenFactorPose2(key, key+1, Pose2(v_del_x, v_del_y, v_del_theta), velocity_model_noise));

%         Calling Scan Matcher and filtering for some modes
%% LASER SCAN MATCHING CONSTRAINTS

        if scan_matching_mode == 1 %  icpmatlab
            [scan_match_R, scan_match_T] = scan_matcher(robot.laser(i).range, robot.laser(i+1).range);
            if norm(scan_match_T) < 0.1 && ~isempty(scan_match_R) && ~isempty(scan_match_T)
                scan_matching_theta = acos(scan_match_R(1,1));
                graph.add(BetweenFactorPose2(key, key+1, ...
                    Pose2(scan_match_T(1), scan_match_T(2), scan_matching_theta), scan_matching_noise));
            end
        elseif scan_matching_mode == 2 % csm_icp_matlab
            [scan_match_R, scan_match_T] = csm_scan_matcher(robot.laser(i).range, robot.laser(i+1).range);
            if norm(scan_match_T) < 0.1 && ~isempty(scan_match_R) && ~isempty(scan_match_T)
                scan_matching_theta = acos(scan_match_R(1,1));
                graph.add(BetweenFactorPose2(key, key+1, ...
                    Pose2(scan_match_T(1), scan_match_T(2), scan_matching_theta), scan_matching_noise));
            end
        elseif scan_matching_mode == 3 % csm_icp_c++
            [init_x, init_y, init_theta] = ...
                odometry_difference(pose_x(i), pose_y(i), pose_theta(i), ...
                                    pose_x(i+1), pose_y(i+1), pose_theta(i+1));

            [scan_match_R, scan_match_T, scan_matching_noise, scan_matching_theta] = ...
                fast_csm_scan_matcher(robot.laser(i).measurement_time, robot.laser(i).range, ...
                robot.laser(i).measurement_time, robot.laser(i+1).range, [init_x, init_y, init_theta]);
            
            if ~isempty(scan_match_R) && ~isempty(scan_match_T)
                scan_matching_noise = noiseModel.Gaussian.Covariance(reshape(scan_matching_noise, [3 3]));
                graph.add(BetweenFactorPose2(key, key+1, ...
                    Pose2(scan_match_T(1), scan_match_T(2), scan_matching_theta), scan_matching_noise));
            end
        end


%% FIDUCIAL CONSTRAINTS 
            
        if ~isempty(robot.fiducial(i).id)
            for j = 1:length(robot.fiducial(i).id)
                if robot.fiducial(i).id(j) > 3 %|| robot.fiducial(i).id(j) == -1
                    [fid_del_x, fid_del_y, fid_del_theta] = ...
                        fiducial_processor(robot.fiducial.range, robot.fiducial.bearing);
                    landmark_key = symbol('l', robot.fiducial(i).id(j));
                    graph.add(BetweenFactorPose2(key, landmark_key, ...
                        Pose2(fid_del_x, fid_del_y, fid_del_theta), fiducial_noise));
                    
                    
                end
            end
        end
        
        
%%
        key = key + 1;
%% INITIAL GUESS
%         Adding initial guess.
        [v_del_x, v_del_y, v_del_theta] = velocity_motion_model(lin_vel(i), ang_vel(i), pose_theta(i), del_t(i));
        init_x = init_x + v_del_x;
        init_y = init_y + v_del_y;
        init_theta = init_theta + v_del_theta;
        initial.insert(key, Pose2(init_x, init_y, init_theta));
    end
end

% Optimize using Levenberg-Marquardt optimization and get marginals
optimizer = LevenbergMarquardtOptimizer(graph, initial);
result = optimizer.optimizeSafely;