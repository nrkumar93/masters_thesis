function [] = fiducial_diagnostics(datamat)

robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

data_size = length(robot.fiducial);

timestep = [];
fiducials = [];

for i = 1:data_size
    for j = robot.fiducial(i).id 
        fiducials = [fiducials j];
        timestep = [timestep i];
    end
end

plot(timestep, fiducials, 'r+');
