clear;

import gtsam.*

robot_activation_mask = [0 1 1 0];
robot_data_end = [nan 13000 inf nan];

data = extract_data(robot_activation_mask, robot_data_end);

params = load_params(robot_activation_mask);

setup_covariances(robot_activation_mask, data, params);




