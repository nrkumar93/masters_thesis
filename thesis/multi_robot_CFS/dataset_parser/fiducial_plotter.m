function [] = fiducial_plotter(datamat, lmap) % Should be changed to load the datamat inside.

if nargin == 1
    mode_lmap = 1;
    mode_odom = 0;
elseif lmap == 1
    mode_lmap = 1;
    mode_odom = 0;
elseif lmap == 0
    mode_lmap = 0;
    mode_odom = 1;
end

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

lmap_poses = [];
lmap_poses = [lmap_poses, robot.lmap.pose];
lmap_x = lmap_poses(1:3:end);
lmap_y = lmap_poses(2:3:end);
lmap_theta = lmap_poses(3:3:end);


assert(mode_lmap == ~mode_odom);

fid = [];
for i = 1:data_size 
    if ~isempty(robot.fiducial(i).id) && ((mode_lmap && ~isempty(robot.lmap(i).pose)) || (mode_odom && ~isempty(robot.odom(i).pose)))
        for j = 1:length(robot.fiducial(i).id)
            if robot.fiducial(i).id(j) == 1 %|| robot.fiducial(i).id(j) == -1
                local_fiducial_x = robot.fiducial(i).range(j) * cos(robot.fiducial(i).bearing(j));
                local_fiducial_y = robot.fiducial(i).range(j) * sin(robot.fiducial(i).bearing(j));
                if mode_lmap
                    fid = [fid; [i (([robot.lmap(i).pose(1); robot.lmap(i).pose(2)]) + rot(robot.lmap(i).pose(3)) * [local_fiducial_x; local_fiducial_y])']];
                elseif mode_odom
                    fid = [fid; [i (([robot.odom(i).pose(1); robot.odom(i).pose(2)]) + rot(robot.odom(i).pose(3)) * [local_fiducial_x; local_fiducial_y])']];
                end
%                 fid = [fid ; [i pose_x(i) + robot.fiducial(i).range(j) * cos(robot.fiducial(i).bearing(j)) ...
%                                 pose_y(i) + robot.fiducial(i).range(j) * sin(robot.fiducial(i).bearing(j))]];
            end
        end
    end
end
    
figure;
hold on;
plot(lmap_x, lmap_y);
plot(fid(:,2), fid(:,3), 'r*');
line([lmap_x(fid(:,1)); fid(:,2)'], [lmap_y(fid(:,1)); fid(:,3)'], 'Color', 'black');
