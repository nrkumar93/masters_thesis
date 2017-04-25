clear;

import gtsam.*

multi_robot_mode = true;

robot_activation_mask = [0 1 1 0];
robot_data_end = [nan 13000 inf nan];

data = extract_data(robot_activation_mask, robot_data_end);

params = load_params(robot_activation_mask);

setup_covariances(robot_activation_mask, data, params);

setup_globals();

% The constraint digit
% 1st digit: cooperative mapping
% 2nd digit: odom
% 3rd digit: velocity model
% 4th digit: lmap
% 5th digit: scan matching
% 6th digit: fiducials
% 7th digit: loop closure

constraints_code = zeros(length(robot_activation_mask), 7);
for i = 1:length(robot_activation_mask)
    constraints_code(i) = [multi_robot_mode, ...
                       params(i).odometry_flag, ...
                       params(i).velocity_model_flag, ...
                       params(i).lmap_flag, ...
                       params(i).scan_matching_flag, ...
                       params(i).fiducial_flag, ...
                       params(i).loop_closure_flag];
end

constraints_code = bitand(bitand(bitand(constraints_code(1,:), ...
                                  constraints_code(2,:)), ...
                                  constraints_code(3,:)), ...
                                  constraints_code(4,:));

constraints_code = bi2de(constraints_code);
