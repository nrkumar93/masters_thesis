function [pose_x, pose_y, pose_theta] = pose_reconstruction_from_scan(datamat)

robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

data_size = length(robot.laser);

Trans_wrt_origin = zeros(3, data_size-1);
Rot_wrt_origin = zeros(3, 3, data_size-1);

i=2;
% k=i-1;

p=2;

% Preallocating the initial Rotation and Translation matrix to avoid check
% inside the loop.
range_1 = robot.laser(i-1).range;
range_2 = robot.laser(i).range;
[Rot_wrt_origin(:,:,1), Trans_wrt_origin(:,1)] = scan_matcher(range_1, range_2);

hold on;

while i <= data_size-1
    for j = i+1:data_size-1
        if ~isempty(robot.laser(j).range)
            break;
        end
    end
    range_1 = robot.laser(i).range;
    range_2 = robot.laser(j).range;
    [R, T] = scan_matcher(range_1, range_2);
    
    Rot_wrt_origin(:,:,p) = Rot_wrt_origin(:,:,p-1) * R;
    Trans_wrt_origin(:,p) = (Rot_wrt_origin(:,:,p-1) * T) + Trans_wrt_origin(:,p-1);
    
    plot(robot.odom(i).pose(1), robot.odom(i).pose(2), '-o');
    plot(Trans_wrt_origin(1,p), Trans_wrt_origin(2,p), 'r*');
%     plot(T(1), T(2), 'r+');
    drawnow;
    
    p=p+1;
%     k=i;
    i=j;
end

% hold off

pose_x = Trans_wrt_origin(1,:);
pose_y = Trans_wrt_origin(2,:);
pose_theta = acos(Rot_wrt_origin(1,1,:));

