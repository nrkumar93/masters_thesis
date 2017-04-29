function params = load_params(robot_activation_mask)

% TODO: Move these params to a modular SAM parameter file.
isam_update_rate = ones(length(robot_activation_mask),1) * 5;
batch_initialization = ones(length(robot_activation_mask),1);
batch_update_size = ones(length(robot_activation_mask),1) * 200; % minimum number of range measurements to process initially
key_offset = 1000000;

for i = 1:length(robot_activation_mask)
    if robot_activation_mask(i) ~= 0
        json_filename = strcat('parameters_iSAM_robot', string(i), '.json');
        unit_params = loadjson(char(json_filename));
    
        %%%%%%%%% Scan Matching Mode %%%%%%%%%
        % 1. icpmatlab
        % 2. csm_icp_matlab
        % 3. csm_icp_c++
        %%%%%%%%% Keyframe Mode %%%%%%%%%
        % 1. intra frame factors
        % 2. fixed lag factors
        % 3. exhaustive factors
        params(i).scan_matching_mode = unit_params{1}.scan_matching.MODE;
        if unit_params{1}.scan_matching.KEYFRAME
            params(i).key_frame_rate = unit_params{1}.scan_matching.KEYFRAME_RATE;
            params(i).key_frame_mode = unit_params{1}.scan_matching.KEYFRAME_MODE;
        else
            params(i).key_frame_rate = 1;
            params(i).key_frame_mode = unit_params{1}.scan_matching.KEYFRAME_MODE;
        end
        
        %%%%%%%%% Type of constraints to be added %%%%%%%%%
        params(i).scan_matching_flag = unit_params{2}.constraints.SCAN_MATCHING_CONSTRAINTS;
        params(i).odometry_flag = unit_params{2}.constraints.ODOMETRY_CONSTRAINTS;
        params(i).velocity_model_flag = unit_params{2}.constraints.VELOCITY_MODEL_CONSTRAINTS;
        params(i).fiducial_flag = unit_params{2}.constraints.FIDUCIAL_CONSTRAINTS;
        params(i).lmap_flag = unit_params{2}.constraints.LMAP_CONSTRAINTS;
        params(i).loop_closure_flag = unit_params{2}.constraints.LOOP_CLOSURE_CONSTRAINTS;


        %%%%%%%%% Parsing the noise models from JSON %%%%%%%%%
        params(i).scan_matching_covariance = unit_params{3}.covariance.SCAN_MATCHING;
        params(i).odometry_covariance = unit_params{3}.covariance.ODOMETRY;
        params(i).velocity_model_covariance = unit_params{3}.covariance.VELOCITY_MODEL;
        params(i).fiducial_covariance = unit_params{3}.covariance.FIDUCIAL;
        params(i).lmap_covariance = unit_params{3}.covariance.LMAP;
        params(i).loop_closure_covariance = unit_params{3}.covariance.LOOP_CLOSURE;
        params(i).encounter_covariance = unit_params{3}.covariance.ENCOUNTER;
        
        params(i).isam_update_rate = isam_update_rate(i);
        params(i).batch_initialization = batch_initialization(i);
        params(i).batch_update_size = batch_update_size(i);
        params(i).key_offset = key_offset;
    end
end

