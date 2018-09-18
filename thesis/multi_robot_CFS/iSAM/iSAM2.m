clearvars -except smoth;

import gtsam.*

%% Load the saved robot specific dataset.
datamat = './data/dataset_robot4.mat';
robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

if robot.odom(1).robot_id == 1
    data_start_point = 3500;
    data_end_point = 12500;
elseif robot.odom(1).robot_id == 2
    data_start_point = 3900;
    data_end_point = 13000;
elseif robot.odom(1).robot_id == 3
    data_start_point = 4000;
    data_end_point = length(robot.odom);
elseif robot.odom(1).robot_id == 4
    data_start_point = 4100;
    data_end_point = 11000;
end

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

%% Loading lmap from the data. DO NOT USE THESE. INDICES CRUNCHED.
lmap_poses = [];
lmap_poses = [lmap_poses, robot.lmap.pose];
lmap_x = lmap_poses(1:3:end);
lmap_y = lmap_poses(2:3:end);
lmap_theta = lmap_poses(3:3:end);

%% Load loop closures
closures = robot.lclosures;
% closures = [closures(1:2,:); closures(3:4,:)];

%% The JSON params and other params.
if robot.odom(1).robot_id == 1
    params = loadjson('parameters_iSAM_robot1.json');
elseif robot.odom(1).robot_id == 2
    params = loadjson('parameters_iSAM_robot2.json');
elseif robot.odom(1).robot_id == 3
    params = loadjson('parameters_iSAM_robot3.json');
elseif robot.odom(1).robot_id == 4
    params = loadjson('parameters_iSAM_robot4.json');
end

batchInitialization=true;
batch_update_size = 200; % minimum number of range measurements to process initially
isam_update_rate = 5;

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
fiducial_flag = params{2}.constraints.FIDUCIAL_CONSTRAINTS;
lmap_flag = params{2}.constraints.LMAP_CONSTRAINTS;
loop_closure_flag = params{2}.constraints.LOOP_CLOSURE_CONSTRAINTS;


%% Parsing the noise models from JSON
scan_matching_covariance = params{3}.covariance.SCAN_MATCHING;
odometry_covariance = params{3}.covariance.ODOMETRY;
velocity_model_covariance = params{3}.covariance.VELOCITY_MODEL;
fiducial_covariance = params{3}.covariance.FIDUCIAL;
lmap_covariance = params{3}.covariance.LMAP;
loop_closure_covariance = params{3}.covariance.LOOP_CLOSURE;

%% Time scaled covariance
odometry_covariance_per_time_ratio = odometry_covariance/(avg_del_t*avg_del_t);
velocity_model_covariance_per_time_ratio = velocity_model_covariance/(avg_del_t*avg_del_t);
lmap_covariance_per_time_ratio = lmap_covariance/(avg_del_t*avg_del_t);

%% Initialize iSAM and Non linear Factor Graph
isam = ISAM2;
graph = NonlinearFactorGraph;
result = Values;

key =1;
count = 0;

%% Initial Estimates

initial = Values;
% init_x = 0.0; init_y = 0.0; init_theta = 3.142;
% initial.insert(key, Pose2(init_x, init_y, init_theta));

%% Adding prior with arbitrary prior covariance.
% priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
% graph.add(PriorFactorPose2(key, Pose2(0, 0, 3.142), priorNoise));

priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
if robot.odom(1).robot_id == 1
    graph.add(PriorFactorPose2(key, Pose2(-0.4, -0.3, 0), priorNoise));  % Subtracting 0.4m and 0.3m from x_1 and y_1 to align it up with robot 3 map
    init_x = -0.4; init_y = -0.3; init_theta = 0;
    initial.insert(key, Pose2(init_x, init_y, init_theta));
elseif robot.odom(1).robot_id == 2
    graph.add(PriorFactorPose2(key, Pose2(-1.22, 0.61, 0), priorNoise));
    init_x = -1.22; init_y = 0.61; init_theta = 0;
    initial.insert(key, Pose2(init_x, init_y, init_theta));
elseif robot.odom(1).robot_id == 3
    graph.add(PriorFactorPose2(key, Pose2(-2.44, 0, 0), priorNoise));
    init_x = -2.44; init_y = 0; init_theta = 0;
    initial.insert(key, Pose2(init_x, init_y, init_theta));
elseif robot.odom(1).robot_id == 4
    graph.add(PriorFactorPose2(key, Pose2(-4.66, 0.36, 0), priorNoise)); % Subtracting 1m and 0.25m from x_4 and y_4 to align it up with robot 3 map.
    init_x = -4.66; init_y = 0.61; init_theta = 0;
    initial.insert(key, Pose2(init_x, init_y, init_theta));
end

% priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
% if robot.odom(1).robot_id == 1
%     graph.add(PriorFactorPose2(key, Pose2(0, 0, 0), priorNoise));  % Subtracting 0.4m and 0.3m from x_1 and y_1 to align it up with robot 3 map
%     init_x = 0; init_y = 0; init_theta = 0;
%     initial.insert(key, Pose2(init_x, init_y, init_theta));
% elseif robot.odom(1).robot_id == 2
%     graph.add(PriorFactorPose2(key, Pose2(-1.22, 0.61, 0), priorNoise));
%     init_x = -1.22; init_y = 0.61; init_theta = 0;
%     initial.insert(key, Pose2(init_x, init_y, init_theta));
% elseif robot.odom(1).robot_id == 3
%     graph.add(PriorFactorPose2(key, Pose2(-2.44, 0, 0.1), priorNoise));
%     init_x = -2.44; init_y = 0; init_theta = 0.1;
%     initial.insert(key, Pose2(init_x, init_y, init_theta));
% elseif robot.odom(1).robot_id == 4
%     graph.add(PriorFactorPose2(key, Pose2(-3.66, 0.61, 0), priorNoise)); % Subtracting 1m and 0.25m from x_4 and y_4 to align it up with robot 3 map.
%     init_x = -3.66; init_y = 0.61; init_theta = 0;
%     initial.insert(key, Pose2(init_x, init_y, init_theta));
% end


i = 1;
frame_id = 1;
correction_angle = -0.85;
factor_indices = [];
cached_frames_isam = [];

% figure; hold on;
% plt_handle = plot(0);


if scan_matching_flag && odometry_flag && fiducial_flag && loop_closure_flag
    while i <= data_size-end_offset-1
        for j = i+1:data_size-end_offset
            if ~isempty(robot.lmap(j).pose)
                break;
            end
        end
        
%% ODOMETRY DEAD RECKONING CONSTRAINTS
% %         Calculating odometry using dead reckoning
        [o_del_x, o_del_y, o_del_theta] = odometry_difference(pose_x(i), pose_y(i), pose_theta(i), ...
                                                              pose_x(j), pose_y(j), pose_theta(j));
                                                          
%         if j == 11825 && robot.odom(1).robot_id == 2
%             o_del_theta = o_del_theta + correction_angle;
%         end

% Delta time scaled Odometry Dead Reckoning covariance. 
        odom_noise = noiseModel.Diagonal.Sigmas([odometry_covariance_per_time_ratio(1); ...
                                                 odometry_covariance_per_time_ratio(5); ...
                                                 odometry_covariance_per_time_ratio(9)] * (del_t(i)^2)); % TODO: del_t based on j-i
%         Adding odometry contraints to graph
        graph.add(BetweenFactorPose2(i, j, Pose2(o_del_x, o_del_y, o_del_theta), odom_noise));

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
        
%% LMAP CONSTRAINTS
%         Using lmap data
        [del_lmap_x, del_lmap_y, del_lmap_theta] = odometry_difference(robot.lmap(i).pose(1), ...
                                                                       robot.lmap(i).pose(2), ...
                                                                       robot.lmap(i).pose(3), ...
                                                                       robot.lmap(j).pose(1), ...
                                                                       robot.lmap(j).pose(2), ...
                                                                       robot.lmap(j).pose(3));
%         if j == 11825 && robot.odom(1).robot_id == 2
%             del_lmap_theta = del_lmap_theta + correction_angle;
%         end

% Delta time scaled Odometry Dead Reckoning covariance. 
        lmap_noise = noiseModel.Diagonal.Sigmas([lmap_covariance(1); ...
                                                 lmap_covariance(5); ...
                                                 lmap_covariance(9)]);
%         Adding odometry contraints to graph
        graph.add(BetweenFactorPose2(i, j, Pose2(del_lmap_x, del_lmap_y, del_lmap_theta), lmap_noise));
        
%% LASER SCAN MATCHING CONSTRAINTS
% %         Calling Scan Matcher and filtering for some modes
    
        cached_frames_isam = [cached_frames_isam frame_id]; 
        if key_frame_rate == 1
            frame_id = i;
%             frame_key = key;
        elseif fix(frame_id/key_frame_rate) ~= fix(i/key_frame_rate)
            frame_id = i;
%             frame_key = key;
        end
        
        assert(frame_id < j);

        if scan_matching_mode == 1 %  icpmatlab
            [scan_match_R, scan_match_T] = scan_matcher(robot.laser(frame_id).range, robot.laser(j).range);
        elseif scan_matching_mode == 2 % csm_icp_matlab
            [scan_match_R, scan_match_T] = csm_scan_matcher(robot.laser(frame_id).range, robot.laser(j).range);
        elseif scan_matching_mode == 3 % csm_icp_c++
%             [csm_init_x, csm_init_y, csm_init_theta] = ...
%                 odometry_difference(pose_x(frame_id), pose_y(frame_id), pose_theta(frame_id), ...
%                                     pose_x(j), pose_y(j), pose_theta(j));
            [csm_init_x, csm_init_y, csm_init_theta] = odometry_difference(robot.lmap(frame_id).pose(1), ...
                                                                           robot.lmap(frame_id).pose(2), ...
                                                                           robot.lmap(frame_id).pose(3), ...
                                                                           robot.lmap(j).pose(1), ...
                                                                           robot.lmap(j).pose(2), ...
                                                                           robot.lmap(j).pose(3));
            
            [scan_match_R, scan_match_T, scan_matching_noise, scan_matching_theta] = ...
                fast_csm_scan_matcher(robot.laser(frame_id).measurement_time, robot.laser(frame_id).range, ...
                robot.laser(j).measurement_time, robot.laser(j).range, [csm_init_x, csm_init_y, csm_init_theta]);
            
            if ~isempty(scan_match_R) && ~isempty(scan_match_T)
                scan_matching_noise = noiseModel.Gaussian.Covariance(reshape(scan_matching_noise, [3 3]));
            end
        end

%         if j == 11825 && robot.odom(1).robot_id == 2
%             scan_matching_theta = scan_matching_theta + correction_angle;
%         end       
        
%         Throw away bizzare scan match results
        if ~isempty(scan_match_R) && ~isempty(scan_match_T)
            [odom_frame_x, odom_frame_y, ~] =  odometry_difference(robot.lmap(frame_id).pose(1), ...
                                                                   robot.lmap(frame_id).pose(2), ...
                                                                   robot.lmap(frame_id).pose(3), ...
                                                                   robot.lmap(j).pose(1), ...
                                                                   robot.lmap(j).pose(2), ...
                                                                   robot.lmap(j).pose(3));
            disparity = abs(norm([odom_frame_x, odom_frame_y]) - norm(scan_match_T));
            
            if disparity < (0.01 * key_frame_rate) || norm(scan_match_T) < (key_frame_rate * 0.1)
                graph.add(BetweenFactorPose2(frame_id, j, ...
                    Pose2(scan_match_T(1), scan_match_T(2), scan_matching_theta), scan_matching_noise)); 
            end
        end

%% FIDUCIAL CONSTRAINTS 
            
        if ~isempty(robot.fiducial(i).id)
            for ele = 1:length(robot.fiducial(i).id)
                if robot.fiducial(i).id(ele) > 3 %|| robot.fiducial(i).id(j) == -1
                    [fid_del_x, fid_del_y, fid_del_theta] = ...
                        fiducial_processor(robot.fiducial(i).range(ele), robot.fiducial(i).bearing(ele));

                    del_fid = Pose2(fid_del_x, fid_del_y, fid_del_theta);
                    landmark_key = symbol('l', robot.fiducial(i).id(ele));
                    
                    % Fiducial covariance. Very low to assert almost zero error.
                    fiducial_noise = noiseModel.Diagonal.Sigmas([fiducial_covariance(1); ...
                                                                 fiducial_covariance(5); ...
                                                                 fiducial_covariance(9)]);
                    
                    graph.add(BetweenFactorPose2(i, landmark_key, del_fid, fiducial_noise));
                    
                    if ~initial.exists(landmark_key) && ~result.exists(landmark_key)
                        fid_init_point2 = (([init_x; init_y]) + rot(init_theta) * [fid_del_x; fid_del_y])';
                        initial.insert(landmark_key, Pose2(fid_init_point2(1), fid_init_point2(2), 0));
                    elseif result.exists(landmark_key) && (i > batch_update_size)
                        estimated_global_pose2 = Pose2(init_x, init_y, init_theta);
                        graph.add(BetweenFactorPose2(i, landmark_key, ...
                            estimated_global_pose2.between(result.at(landmark_key)), noiseModel.Diagonal.Sigmas([0.00001; 0.00001; 0.00001])));                            
                    end                
                end
            end
        end
        
%% LOOP CLOSURE CONSTRAINTS

% %         I AM NOT CHECKING IF THE ENTRIES OF ACCESSING LMAP FOR LOOP
% %         CLOSURE CONSTRAINTS ARE NON EMPTY BECAUSE WE ARRIVED AT THESE
% %         LOOP CLOSURE NUMBERS BASED ON LMAP.
        [~, loc] = ismember(i, closures(:,1));
        if loc ~= 0
            [del_lc_x, del_lc_y, del_lc_theta] = odometry_difference(robot.lmap(i).pose(1), ...
                                                                     robot.lmap(i).pose(2), ...
                                                                     robot.lmap(i).pose(3), ...
                                                                     robot.lmap(closures(loc,2)).pose(1), ...
                                                                     robot.lmap(closures(loc,2)).pose(2), ...
                                                                     robot.lmap(closures(loc,2)).pose(3));
            del_lc = Pose2(del_lc_x, del_lc_y, del_lc_theta);
%             del_lc = Pose2(0, 0, del_lc_theta);
            % Loop closure covariance. Very low to assert almost zero error.
            loop_closure_noise = noiseModel.Diagonal.Sigmas([loop_closure_covariance(1); ...
                                                            loop_closure_covariance(5); ...
                                                            loop_closure_covariance(9)]);
            graph.add(BetweenFactorPose2(i, closures(loc,2), del_lc, loop_closure_noise));
        end
              
        
        
%%
        key = key + 1;
        count = count + 1;
        
%% INITIAL GUESS
%         Adding initial guess.
%         [v_del_x, v_del_y, v_del_theta] = velocity_motion_model(lin_vel(i), ang_vel(i), pose_theta(i), del_t(i));

%         [v_del_x, v_del_y, v_del_theta] = odometry_difference(pose_x(i), pose_y(i), pose_theta(i), ...
%                                                               pose_x(j), pose_y(j), pose_theta(j));
          [v_del_x, v_del_y, v_del_theta] = odometry_difference(robot.lmap(i).pose(1), ...
                                                               robot.lmap(i).pose(2), ...
                                                               robot.lmap(i).pose(3), ...
                                                               robot.lmap(j).pose(1), ...
                                                               robot.lmap(j).pose(2), ...
                                                               robot.lmap(j).pose(3));

        init_x = init_x + v_del_x;
        init_y = init_y + v_del_y;
        init_theta = init_theta + v_del_theta;
%         if j == 11825 && robot.odom(1).robot_id == 2
%             init_theta = init_theta + correction_angle;
%         end        
        initial.insert(j, Pose2(init_x, init_y, init_theta));
        
%         initial.insert(j, Pose2(robot.lmap(j).pose(1), robot.lmap(j).pose(2), robot.lmap(j).pose(3)));

        if i > batch_update_size && count > isam_update_rate
            if batchInitialization
                batchOptimizer = LevenbergMarquardtOptimizer(graph, initial);
                initial = batchOptimizer.optimize();
                batchInitialization = false; % only once                
            end
            tic
            isam.update(graph, initial);
%             plt_handle.YData = [plt_handle.YData toc];
            result = isam.calculateEstimate();
            init_x = result.at(j).x;
            init_y = result.at(j).y;
            init_theta = result.at(j).theta;
            graph = NonlinearFactorGraph;
            initial = Values;
            count = 0;            
            drawnow;
        end
        
        factor_indices = [factor_indices i];
        i = j;
    end
end

% result
isam.update(graph, initial);
result = isam.calculateEstimate();
XYT = utilities.extractPose2(result);

% figure; hold on;
% plot(XYT(:,1),XYT(:,2),'k-');

% Retrieving the smoothed data.
smoothed_results = [];
for k = factor_indices
    smoothed_results = [smoothed_results; [result.at(k).x, result.at(k).y, result.at(k).theta]];
end

% Plotting

figure; hold on;
plot(smoothed_results(:,1), smoothed_results(:,2));

