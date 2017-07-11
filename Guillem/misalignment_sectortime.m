function [ tindex_lost, sector_lost ] = misalignment_sectortime( beamwidth, dist_max, ang_max, avg_velocity, std_deviation_velocity, number_of_samples, doppler_res,t)

% This function returns the index of the vector t where
%misalignment happened (ts) and the sector where misalingment happended (s)


%% Calculation of the sectors

%sector_limits indicates each border between two sectors
sector_limits = calc_sectors(beamwidth, dist_max, ang_max);


%% Velocities and error data generation

%velocities are created with a Gaussian distr with mean=avg_velocity and
%standard deviation = std_deviation_velocity. Doppler resolution is
%distributed uniformly from 0 to Doppler resolution

%velocity_data returns two vectors with 'number_of_samples' samples of the
%random generated velocities and errors
[velocities, errors] = velocity_data(avg_velocity, std_deviation_velocity, number_of_samples, doppler_res);

%Error is added to the velocities
velocities_error=velocities+errors;

%% Position calculation on every time instant

%positions on every time instant for both the estimated and real velocity are calculated
positions=velocities*t;
positions_error=velocities_error*t;

%% Calculation of the sector each car is in every time instant for both real and estimated velocities


[sectors, sectors_error] = sectors_t( positions, positions_error, sector_limits, velocities);

%Misalingment is calculated comparing the sector of each position for the
%real and estimated velocity
err=(sectors==sectors_error);

%% Calculation of the sector and the instant of the first misalignment 

%misalingnent_instants returns the index in vector t where the first
%misalingment happened and the sector where first misalignment happened.
%The sector returned for each far is the one where the car was EXPECTED to
%be not the one where actually is
[ tindex_lost, sector_lost ] = misalignment_instants( err, sectors_error, t, sector_limits );



end

