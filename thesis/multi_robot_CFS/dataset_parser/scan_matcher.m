function [R, T] = scan_matcher(range_1, range_2)

% Function to prepare data and call ICP.

% Return if no LIDAR data is recorded while logging a odom data.
if isempty(range_1) || isempty(range_2)
    R = [];
    T = [];
    return
end

% Caching the sine and cosine values for fast access.
sine_cache = evalin('base', 'sine_cache');
cosine_cache = evalin('base', 'cosine_cache');

% Converting to Cartesian coordinates.
x_1 = range_1 .* cosine_cache;
y_1 = range_1 .* sine_cache;

x_2 = range_2 .* cosine_cache;
y_2 = range_2 .* sine_cache;

% Calling ICP
[R, T] = icpmatlab([x_1; y_1; zeros(1,181)], [x_2; y_2; zeros(1,181)]);
