function [ tindex_lost, sector_lost ] = misalignment_sectortime10( beamwidth, dist_max, ang_max, avg_velocity, std_deviation_velocity, number_of_samples, doppler_res,t)

% The function returns the index of the vector t where misalignment 
% happened (ts) and the sector where misalingment happended (s)

%% Calculation of the sectors
% Return the boundaries in meters of each sector. The origin is at the edge
% of the coverage area.
sector_limits = calc_sectors(beamwidth, dist_max, ang_max);

%% Radar angular sector, 
% sectors that belong to the radar angular sector are removed so that 
% we analyze the error starting at the end of the radar angular sector.
red = floor(10/beamwidth);
sector_limits(1:red) = [];
sector_limits = sector_limits - sector_limits(1);

%% Velocities and error data generation

% velocities are created with a Gaussian distr with mean=avg_velocity and
% standard deviation = std_deviation_velocity. Doppler resolution is
% distributed uniformly [0 , doppler_res]

% velocity_data returns two vectors with 'number_of_samples' samples of the
% random generated velocities and errors
[velocities, errors] = velocity_data(avg_velocity, std_deviation_velocity, number_of_samples, doppler_res);

% Error is added to the velocities
velocities_error = velocities + errors;

%% Position calculation on every time instant

% Positions on every time instant for both the estimated and real velocity are calculated
positions = velocities*t;
positions_error = velocities_error*t;

%% Calculation of the sector each car is in every time instant for both real and estimated velocities

[sectors, sectors_error] = sectors_t( positions, positions_error, sector_limits, velocities);

% Misalingment is calculated comparing the sector of each position for the
% real and estimated velocity
err = (sectors==sectors_error);

%% Calculation of the sector and the instant of the first misalignment 

% misalingnent_instants returns the index in vector t where the first
% misalingment happened and the sector where first misalignment happened.
% The sector returned for each far is the one where the car was EXPECTED to
% be not the one where actually is
[ tindex_lost, sector_lost ] = misalignment_instants( err, sectors_error, t, sector_limits );

end
