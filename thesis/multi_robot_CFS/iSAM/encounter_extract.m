datamat = './data/dataset_robot1.mat';
robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

if robot.odom(1).robot_id == 1
    data_start_point = 3500;
    data_end_point = 12500;
elseif robot.odom(1).robot_id == 2
    data_start_point = 3900;
    data_end_point = 13000;
elseif robot.odom(1).robot_id == 3
    data_start_point = 4000;
    data_end_point = length(robot.odom);
elseif robot.odom(1).robot_id == 4
    data_start_point = 4100;
    data_end_point = 11000;
end

robot = data_chopper(robot, data_start_point, data_end_point);

data_size = length(robot.odom);
start_offset = 0;
end_offset = 0;

i = 1;
count = 0;
encounter = struct('target_fid', {}, 'factor_indices', {});

while i <= data_size-end_offset-1
    for j = i+1:data_size-end_offset
        if ~isempty(robot.lmap(j).pose)
            break;
        end
    end
    
    if ~isempty(robot.fiducial(i).id)
        flag = 1;
        for ele = 1:length(robot.fiducial(i).id)
            if robot.fiducial(i).id(ele) == 0 || ...
               robot.fiducial(i).id(ele) == 1 || ...
               robot.fiducial(i).id(ele) == 2 || ...
               robot.fiducial(i).id(ele) == 3
                if flag == 1
                    count = count + 1;
                    encounter(count).target_fid = [];
                    flag = 0;
                end
                
                encounter(count).factor_indices =  i;
                encounter(count).target_fid = [encounter(count).target_fid robot.fiducial(i).id(ele)+1];
            end
        end
    end
    
    
    i = j;
end
