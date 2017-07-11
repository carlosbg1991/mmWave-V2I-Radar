function [ velocities, errors ] = velocity_data( avg, std_dev, N, doppler_res )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
velocity_gauss=gmdistribution(avg, std_dev^2);
velocities=random(velocity_gauss,N);
velocities=velocities*1000/3600;

errors=doppler_res*rand(N,1);
end

