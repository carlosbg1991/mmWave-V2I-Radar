function [ sector_limits ] = calc_sectors( beamwidth, dist_max, ang_max )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

half_ang_max=ang_max/2;

angles=half_ang_max:-beamwidth:-half_ang_max;

dist_middle_point=dist_max*cosd(half_ang_max);

sector_limits=dist_middle_point*(tand(60)-tand(angles));

end

