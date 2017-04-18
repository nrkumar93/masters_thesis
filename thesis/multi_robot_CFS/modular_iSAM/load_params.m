function params = load_params(robot_activation_mask)

for i = 1:length(robot_activation_mask)
    if robot_activation_mask(i) ~= 0
        json_filename = strcat('parameters_iSAM_robot', string(i), '.json');
        unit_params = loadjson(char(json_filename));
    
        %%%%%%%%% Scan Matching Mode %%%%%%%%%
        % 1. icpmatlab
        % 2. csm_icp_matlab
        % 3. csm_icp_c++
        params(i).scan_matching_mode = unit_params{1}.scan_matching.MODE;
        if unit_params{1}.scan_matching.KEYFRAME
            params(i).key_frame_rate = unit_params{1}.scan_matching.KEYFRAME_RATE;
        else
            params(i).key_frame_rate = 1;
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
    end
end

