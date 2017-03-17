function [fiducials] = fiducial_plotter(loaded_datamat) % Should be changed to load the datamat inside.

offline = 1;

data_size = length(loaded_datamat.fiducial);

fiducials = [];

figure;
hold on;
for i = 1:data_size
    for j = 1:length(loaded_datamat.fiducial(i).id)
        if ~offline
            plot(i, loaded_datamat.fiducial(i).id(j), 'r*');
            drawnow;
        else
            fiducials = [fiducials; [i, loaded_datamat.fiducial(i).id(j)]];
        end
    end
end
