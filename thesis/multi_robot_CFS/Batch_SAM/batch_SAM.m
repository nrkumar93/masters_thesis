function [opt_x, opt_y, opt_theta] = batch_SAM(datamat)

load('sine_cache.mat');
load('cosine_cache.mat');

% Load the saved robot specific dataset.
robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

% The length of the dataset
data_size = length(robot.odom);

% The JSON params 
params = loadjson('parameters.json');



 