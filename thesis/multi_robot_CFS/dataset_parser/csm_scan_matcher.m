function [R, T] = csm_scan_matcher(range_1, range_2)

% Return if no LIDAR data is recorded while logging a odom data.
if isempty(range_1) || isempty(range_2)
    R = [];
    T = [];
    return
end

% Caching the sine and cosine values for fast access.
sine_cache = evalin('base', 'sine_cache');
cosine_cache = evalin('base', 'cosine_cache');

% Function to prepare data and call ICP.
laser_ref.nrays = 181;
laser_ref.theta = linspace(-pi/2, pi/2, 181);
laser_ref.readings = range_1;
laser_ref.points = [cosine_cache .* range_1; sine_cache .* range_1];

% Function to prepare data and call ICP.
laser_sens.nrays = 181;
laser_sens.theta = linspace(-pi/2, pi/2, 181);
laser_sens.readings = range_2;
laser_sens.points = [cosine_cache .* range_2; sine_cache .* range_2];

params.laser_ref = laser_ref;
params.laser_sens = laser_sens;

% Calling ICP
result = icp(params);
T = [result.X(1:2);0];
R = [cos(result.X(3)), -sin(result.X(3)), 0;
     sin(result.X(3)), cos(result.X(3)), 0;
     0, 0, 1];

