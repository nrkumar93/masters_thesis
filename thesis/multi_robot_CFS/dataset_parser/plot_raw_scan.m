function [] = plot_raw_scan(datamat, index)

robot = load(datamat);
robot = getfield(robot, char(fieldnames(robot)));

% The length of the dataset
data_size = length(robot.laser);

plot_offset = 1;

sine_cache = evalin('base', 'sine_cache');
cosine_cache = evalin('base', 'cosine_cache');

if nargin == 2
%     Current Laserscan.
    range_main = robot.laser(index).range;

%     Valid previous Laserscan.
    for j = index-1:-1:1
        if ~isempty(robot.laser(j).range)
            range_before= robot.laser(j).range;
            break;
        end
    end
    
%     Valid next Laserscan.    
    for j = index+1:data_size
        if ~isempty(robot.laser(j).range)
            range_after = robot.laser(j).range;
            break;
        end
    end
    
    x = range_main .* cosine_cache;
    y = range_main .* sine_cache;
    
    x_b = range_before .* cosine_cache;
    y_b = range_before .* sine_cache;
    
    x_a = range_after .* cosine_cache;
    y_a = range_after .* sine_cache;

    figure;
    hold on
    plot(x, y);
    plot(x_b, y_b);
    plot(x_a, y_a);
else
    figure;
    hold on
    for i=plot_offset:data_size
        range_main = robot.laser(i).range;
        x = range_main .* cosine_cache;
        y = range_main .* sine_cache;
        
        plot(x, y, 'b');
    end
end


