function [tindex_lost, beam_switching_positions, dists_covered] = radarAD_misalignment(beamwidth)

%clear all;
close all;
%beam coverage of 120º
%max distance: 200m
%lane width: 3.5m

if mod(120,beamwidth)~=0
    disp("choose a value divisible by 120")
    return
end

max_dist=200;

lane_width=3.5;
lane_widths=lane_width*(1:10);
dist_middle_point=max_dist*cosd(60);
dist_lane= 200-(lane_widths./cosd(60));

%assuming every beamwidth the same (1º)
max_cov =100*(tand(60)-tand(59));
%beamwidth=3;
angles=60:-beamwidth:-60;
beam_switching_positions=100*(tand(60)-tand(angles));

for i=1:length(angles)-1
    dists_covered(i)=dist_middle_point*(tand(angles(i))-tand(angles(i+1)));
end

%velocity range [80,140] km/h  [22.2222,80.140] m/s

velocity_gauss=gmdistribution(110,10^2);
velocities=(1000/3600)*random(velocity_gauss,10000);

figure;
histogram(velocities)
title("Histogram of the velocities generated data")

max_v_error=2;
errors=max_v_error*rand(10000,1);

velocities_error=velocities+errors;

figure;
histogram(velocities_error)
title("Histogram of the velocities with error generated data")

t=0:0.1:15;

positions=velocities*t;
positions_error=velocities_error*t;

sectors=zeros(size(positions));
sectors_error=zeros(size(positions_error));

for i=1:(length(beam_switching_positions)-1)
    
    for j=1:length(velocities)

        s=find(positions(j,:)>beam_switching_positions(i) & positions(j,:)<beam_switching_positions(i+1));
        sectors(j,s)=i;

        s=find(positions_error(j,:)>beam_switching_positions(i) & positions_error(j,:)<beam_switching_positions(i+1));
        sectors_error(j,s)=i;
    end
end


err=(sectors==sectors_error);

tindex_lost=zeros(size(err,1),1);
sector_lost=zeros(size(err,1),1);

for i=1:size(sectors,1)
    
    a=find(err(i,:)==0,1);
    if isempty(a)==false
        tindex_lost(i)=a;
        sector_lost(i)=sectors_error(i,a);
    else
        tindex_lost(i)=length(t);
        sector_lost(i)=length(angles)-1;
    end
end

figure;
histogram(sector_lost)
xlabel("sector")

figure;
histogram(t(tindex_lost)*1000,300)
xlabel("time instant where the first sector error happens [ms]")


avg_misalign_t=mean(t(tindex_lost));

fprintf(1,'Misalingnment happens after %g seconds (average)  \n', avg_misalign_t)

%figure;
%histogram(positions_error(:,tindex_lost))
end
