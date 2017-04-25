function [fid_x, fid_y, fid_theta] = fiducial_processor(range, bearing)

fid_x = range * cos(bearing);
fid_y = range * sin(bearing);
fid_theta = bearing;