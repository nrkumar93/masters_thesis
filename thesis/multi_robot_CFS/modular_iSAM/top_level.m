clear;

import gtsam.*

robot_activation_mask = [0 1 1 0];
robot_data_end = [nan 13000 inf nan];

data = extract_data(robot_activation_mask, robot_data_end);

params = load_params(robot_activation_mask);

global odometry_covariance_per_time_ratio velocity_model_covariance_per_time_ratio lmap_covariance_per_time_ratio;
odometry_covariance_per_time_ratio = zeros(length(robot_activation_mask));
velocity_model_covariance_per_time_ratio = zeros(length(robot_activation_mask));
lmap_covariance_per_time_ratio = zeros(length(robot_activation_mask));

for i = 1:length(robot_activation_mask)
    if robot_activation_mask(i) ~= 0
        odometry_covariance_per_time_ratio(i) = params(i).odometry_covariance/(data(i).avg_del_t*data(i).avg_del_t);
        velocity_model_covariance_per_time_ratio(i) = params(i).velocity_model_covariance/(data(i).avg_del_t*data(i).avg_del_t);
        lmap_covariance_per_time_ratio(i) = params(i).lmap_covariance/(data(i).avg_del_t*data(i).avg_del_t);  
    end    
end

