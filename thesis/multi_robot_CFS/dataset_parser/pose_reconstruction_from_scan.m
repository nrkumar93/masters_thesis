function [pose_x, pose_y, pose_theta] = pose_reconstruction_from_scan(datamat)

robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

data_size = length(robot.laser);

% Scan Matching Mode
% 1. icpmatlab
% 2. csm_icp_matlab
% 3. csm_icp_c++

mode = 3;

% Loading all the odometry poses from the dataset
all_poses = [];
all_poses = [all_poses, robot.odom.pose];
pose_x = all_poses(1:3:end);
pose_y = all_poses(2:3:end);
pose_theta = all_poses(3:3:end);

Trans_wrt_origin = zeros(3, data_size-1);
Rot_wrt_origin = zeros(3, 3, data_size-1);
covariance = zeros(9, data_size-1);

Trans_local = zeros(3, data_size-1);
Rot_local = zeros(3, 3, data_size-1);

i=4000;
offset=1;
% k=i-1;

p=2;

% Preallocating the initial Rotation and Translation matrix to avoid check
% inside the loop.
range_1 = robot.laser(i-1).range;
range_2 = robot.laser(i).range;
if mode == 1
    [Rot_wrt_origin(:,:,1), Trans_wrt_origin(:,1)] = scan_matcher(range_1, range_2);
elseif mode == 2
    [Rot_wrt_origin(:,:,1), Trans_wrt_origin(:,1)] = csm_scan_matcher(range_1, range_2);    
elseif mode == 3
    [init_x, init_y, init_theta] = ...
        odometry_difference(pose_x(i-1), pose_y(i-1), pose_theta(i-1), pose_x(i), pose_y(i), pose_theta(i));
    
    [Rot_wrt_origin(:,:,1), Trans_wrt_origin(:,1), covariance(:,1)] = ...
        fast_csm_scan_matcher(robot.laser(i-1).measurement_time, range_1, ...
        robot.laser(i).measurement_time, range_2, [init_x, init_y, init_theta]);
end

Trans_local(:,1) = Trans_wrt_origin(:,1);
Rot_local(:,:,1) = Rot_wrt_origin(:,:,1);

data_distance = [];
scan_match_distance = [];

data_distance(1) = distance(robot.odom(i).pose, robot.odom(i-1).pose);
scan_match_distance(1) = sqrt(Trans_wrt_origin(1,1)^2 + Trans_wrt_origin(2,1)^2);

while i <= data_size-offset
    for j = i+offset:data_size
        if ~isempty(robot.laser(j).range)
            break;
        end
    end
    range_1 = robot.laser(i).range;
    range_2 = robot.laser(j).range;
    if mode == 1
        [R, T] = scan_matcher(range_1, range_2);
    elseif mode == 2
        [R, T] = csm_scan_matcher(range_1, range_2);
    elseif mode == 3
        [init_x, init_y, init_theta] = ...
            odometry_difference(pose_x(i), pose_y(i), pose_theta(i), pose_x(j), pose_y(j), pose_theta(j));

        [R, T, covariance(:,p)] = ...
            fast_csm_scan_matcher(robot.laser(i).measurement_time, range_1, ...
            robot.laser(j).measurement_time, range_2, [init_x, init_y, init_theta]);
    end
    
    Trans_local(:, p) = T;
    Rot_local(:, :, p) = R;
    
    data_distance(p) = distance(robot.odom(i).pose, robot.odom(j).pose);
    scan_match_distance(p) = sqrt(T(1)^2 + T(2)^2);
    
    
    Rot_wrt_origin(:,:,p) = Rot_wrt_origin(:,:,p-1) * R;
    Trans_wrt_origin(:,p) = (Rot_wrt_origin(:,:,p-1) * T) + Trans_wrt_origin(:,p-1);
    
%     plot(robot.odom(i).pose(1), robot.odom(i).pose(2), '-o');
%     plot(Trans_wrt_origin(1,p), Trans_wrt_origin(2,p), 'r*');
%     plot(T(1), T(2), 'r+');
    drawnow;
    
    p=p+1;
%     k=i;
    i=j;
end

figure;
plot(data_distance);
hold on;
scan_match_distance(scan_match_distance > 0.14) = 0;
plot(scan_match_distance);
% hold off

figure;
plot(data_distance - scan_match_distance);

pose_x = Trans_wrt_origin(1,:);
pose_y = Trans_wrt_origin(2,:);
pose_theta = acos(Rot_wrt_origin(1,1,:));

function dist = distance(A, B)

dist = sqrt((A(1) - B(1))^2 + (A(2) - B(2))^2);
