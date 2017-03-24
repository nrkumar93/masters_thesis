function [] = fiducial_plotter(datamat) % Should be changed to load the datamat inside.

offline = 1;

robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

data_size = length(robot.fiducial);

% fiducials = [];
% 
% figure;
% hold on;
% for i = 1:data_size
%     for j = 1:length(robot.fiducial(i).id)
%         if ~offline
%             plot(i, robot.fiducial(i).id(j), 'r*');
%             drawnow;
%         else
%             fiducials = [fiducials; [i, robot.fiducial(i).id(j)]];
%         end
%     end
% end
% 
% if offline
%     plot(fiducials(:,1), fiducials(:,2), 'r*');
% end


% Loading all the odometry model values from the data.
all_poses = [];
all_poses = [all_poses, robot.odom.pose];
pose_x = all_poses(1:3:end);
pose_y = all_poses(2:3:end);
pose_theta = all_poses(3:3:end);

fid = [];
for i = 1:data_size 
    if ~isempty(robot.fiducial(i).id)
        for j = 1:length(robot.fiducial(i).id)
            if robot.fiducial(i).id(j) > 3 %|| robot.fiducial(i).id(j) == -1
                local_fiducial_x = robot.fiducial(i).range(j) * cos(robot.fiducial(i).bearing(j));
                local_fiducial_y = robot.fiducial(i).range(j) * sin(robot.fiducial(i).bearing(j));
                fid = [fid; [i (([pose_x(i); pose_y(i)]) + rot(pose_theta(i)) * [local_fiducial_x; local_fiducial_y])']];
%                 fid = [fid ; [i pose_x(i) + robot.fiducial(i).range(j) * cos(robot.fiducial(i).bearing(j)) ...
%                                 pose_y(i) + robot.fiducial(i).range(j) * sin(robot.fiducial(i).bearing(j))]];
            end
        end
    end
end
    
figure;
hold on;
plot(pose_x, pose_y);
plot(fid(:,2), fid(:,3), 'r*');
line([pose_x(fid(:,1)); fid(:,2)'], [pose_y(fid(:,1)); fid(:,3)'], 'Color', 'black');
