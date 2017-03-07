function [R, T] = scan_matcher(range_1, range_2)

if isempty(range_1) || isempty(range_2)
    R = [];
    T = [];
    return
end

sine_cache = evalin('base', 'sine_cache');
cosine_cache = evalin('base', 'cosine_cache');

x_1 = range_1 .* cosine_cache;
y_1 = range_1 .* sine_cache;

x_2 = range_2 .* cosine_cache;
y_2 = range_2 .* sine_cache;

[R, T] = icp([x_1; y_1; zeros(1,181)], [x_2; y_2; zeros(1,181)]);
