red = imread('red_gra_resize.png');
blue = imread('blue_gra_resize.png');

bw_red = im2bw(red,0.5);
red_val = zeros(1,size(bw_red,2));
for i = 1:size(bw_red,2)
    for j = 1:size(bw_red,1)
        if bw_red(j,i) == 0
            red_val(i) = size(bw_red,1) - j;
            break;
        end
    end
end

bw_blue = im2bw(blue,0.5);
blue_val = zeros(1,size(bw_blue,2));
for i = 1:size(bw_blue,2)
    for j = 1:size(bw_blue,1)
        if bw_blue(j,i) == 0
            blue_val(i) = size(bw_blue,1) - j;
            break;
        end
    end
end

scale = size(bw_red,1)/400;
red_val = red_val/scale;
blue_val = blue_val/scale;

figure; hold on;
plot(blue_val, 'b-', 'LineWidth', 1)
stem(red_val, 'r', 'Marker', 'none')
xlim([0 size(bw_red,2)])
ylim([0 size(bw_red,1)/scale])
