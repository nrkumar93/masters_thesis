
figure; 
hold on;

h = plot3(0,0,0);

h(1).LineStyle = 'none';
h(1).Marker = '.';
h(1).Color = 'k';
h(1).MarkerSize = 10;
% h(2).LineStyle = 'none';
% h(2).Marker = '.';
% h(2).Color = 'r';


for i = 1:1000
    j = [i-500 i];
    h(1).XData = [h(1).XData zeros(1,180)];
    h(1).YData = [h(1).YData (i-500)*ones(1,180)];
    h(1).ZData = [h(1).ZData i*ones(1,180)];
%     h(2).XData = [h(2).XData main_smooth(k,1)];
%     h(2).YData = [h(2).YData main_smooth(k,2)];
%     h(2).ZData = [h(2).ZData k];
    drawnow;    
end
