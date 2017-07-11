function [ tindex_lost, sector_lost ] = misalignment_instants( err, sectors_error, t, sector_limits )
%UNTITLED14 Summary of this function goes here
%   Detailed explanation goes here
tindex_lost=zeros(size(err,1),1);
sector_lost=zeros(size(err,1),1);

for i=1:size(sectors_error,1)
    
    a=find(err(i,:)==0,1);
    if isempty(a)==false
        tindex_lost(i)=a;
        sector_lost(i)=sectors_error(i,a);
    else
        tindex_lost(i)=length(t);
        sector_lost(i)=length(sector_limits)-1;
    end
end


end

