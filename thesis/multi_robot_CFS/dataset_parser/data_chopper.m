function [chopped_dataset] = data_chopper(input_data, lower_limit, upper_limit)

chopped_dataset.odom = input_data.odom(lower_limit:upper_limit);
chopped_dataset.laser = input_data.laser(lower_limit:upper_limit);
chopped_dataset.fiducial = input_data.fiducial(lower_limit:upper_limit);
chopped_dataset.lmap = input_data.lmap(lower_limit:upper_limit);
chopped_dataset.sync = input_data.sync(lower_limit:upper_limit);
if isfield(input_data, 'lclosures')
    chopped_dataset.lclosures = input_data.lclosures;
end

