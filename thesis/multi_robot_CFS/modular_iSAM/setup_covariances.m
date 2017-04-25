function [] = setup_covariances(robot_activation_mask, data, params)

global odometry_covariance_per_time_ratio velocity_model_covariance_per_time_ratio ...
       lmap_covariance_per_time_ratio scan_matching_covariance loop_closure_covariance ...
       fiducial_covariance encounter_covariance;
   
odometry_covariance_per_time_ratio = zeros(length(robot_activation_mask), 9);
velocity_model_covariance_per_time_ratio = zeros(length(robot_activation_mask), 9);
lmap_covariance_per_time_ratio = zeros(length(robot_activation_mask), 9);
scan_matching_covariance = zeros(length(robot_activation_mask), 9);
loop_closure_covariance = zeros(length(robot_activation_mask), 9);
fiducial_covariance = zeros(length(robot_activation_mask), 9);
encounter_covariance = zeros(length(robot_activation_mask), 9);

for i = 1:length(robot_activation_mask)
    if robot_activation_mask(i) ~= 0
        odometry_covariance_per_time_ratio(i,:) = params(i).odometry_covariance/(data(i).avg_del_t*data(i).avg_del_t);
        velocity_model_covariance_per_time_ratio(i,:) = params(i).velocity_model_covariance/(data(i).avg_del_t*data(i).avg_del_t);
        lmap_covariance_per_time_ratio(i,:) = params(i).lmap_covariance/(data(i).avg_del_t*data(i).avg_del_t);  
        scan_matching_covariance(i,:) = params(i).scan_matching_covariance;
        loop_closure_covariance(i,:) = params(i).loop_closure_covariance;
        fiducial_covariance(i,:) = params(i).fiducial_covariance;
        encounter_covariance(i,:) = params(i).encounter_covariance;        
    end    
end


