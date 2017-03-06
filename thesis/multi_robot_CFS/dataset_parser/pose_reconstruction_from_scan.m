function [pose_x, pose_y, pose_theta] = pose_reconstruction_from_scan(datamat)

robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

data_size = length(robot.laser);

Trans_wrt_origin = zeros(3, data_size-1);
Rot_wrt_origin = zeros(3, 3, data_size-1);

% Preallocating the initial Rotation and Translation matrix to avoid check
% inside the loop.
range_1 = robot.laser(1).range;
range_2 = robot.laser(2).range;
[Rot_wrt_origin(:,:,1), Trans_wrt_origin(:,1)] = scan_matcher(range_1, range_2);

k=1;
i=2;
while i < data_size-1
    for j = i+1:data_size-1
        if ~isempty(robot.laser(j).range)
            break;
        end
    end
    range_1 = robot.laser(i).range;
    range_2 = robot.laser(j).range;
    [R, T] = scan_matcher(range_1, range_2);
    
    Rot_wrt_origin(:,:,i) = Rot_wrt_origin(:,:,k) * R;
    Trans_wrt_origin(:,i) = (Rot_wrt_origin(:,:,k) * T) + Trans_wrt_origin(:,k);
    
    k=i;
    i=j;
end

pose_x = Trans_wrt_origin(1,:);
pose_y = Trans_wrt_origin(2,:);
pose_theta = acos(Rot_wrt_origin(1,1,:));

