function [ sectors, sectors_error ] = sectors_t ( positions, positions_error, sector_limits, velocities )
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
sectors = zeros(size(positions));
sectors_error = zeros(size(positions_error));

for i=1:(length(sector_limits)-1)
    
    for j=1:length(velocities)

        s=find(positions(j,:)>sector_limits(i) & positions(j,:)<sector_limits(i+1));
        sectors(j,s)=i;

        s=find(positions_error(j,:)>sector_limits(i) & positions_error(j,:)<sector_limits(i+1));
        sectors_error(j,s)=i;
    end
end


end

