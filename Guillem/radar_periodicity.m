clear all;
close all;

sector_limits = calc_sectors(0.5, 200, 120);


velocity = 120*1000/3600;

%t_angsector is the time a car would need to cross and angular sector from
%0.5º to 120º in ms. This parameter would define the radar periodicity
t_angsector = 1000*sector_limits./velocity;

%t_sweep is the time the radar would need to sweep an angular sector from
%0.5º to 120º in ms
t_sweep = 1.25*(0:1:length(sector_limits)-1);

figure
hold on;
plot(t_sweep, t_angsector)
xlabel("Radar time for sweeping the angular sector [ms]")
ylabel("Time that a car would need to cross the whole are covered by the angular sector [ms]")
title("Time a car crossing and angular sector at 120km/h in terms of radar time application for that same sector")

% 
% for i=1:(length(t_sweep)-1)
%     
%     slope(i)=(t_angsector(i+1)-t_angsector(i))/(t_sweep(i+1)-t_sweep(i));
%     
% end
% 
% theta=0:0.5:119.5;
% figure
% plot(theta,slope)

%This plot represents the ratio [radar perdiodicity - radar time] for each
%angular sector value
theta=0:0.5:120;
figure
plot(theta, t_angsector./t_sweep)
hold on;
xlabel("angular sector width");
ylabel("ratio radar periodicity - radar application time");

