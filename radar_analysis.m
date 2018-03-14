clear; close all; clc;

TSLOT = 32.767/2;
BEAMWIDTH = 10;
rBEAM = 0.5;
maxSpeed = 30;

% Number of Packets for radar
rNPKTList = (10:10:500);

e_radar = zeros(length(rNPKTList),1);
t_radar = zeros(length(rNPKTList),1);
s_radar = zeros(length(rNPKTList),1);
for idx = 1:length(rNPKTList)
rNPKT = rNPKTList(idx);
[e_radar(idx,1),t_radar(idx,1),s_radar(idx,1)] = v2i_conf_radar(maxSpeed,rNPKT,rBEAM,BEAMWIDTH,TSLOT);
end

figure(1);
subplot(2,1,1);
plot(rNPKTList,e_radar,'k-','LineWidth',2);
xlabel('Number of Packets','FontSize',12);
ylabel('Doppler resolution (m/s)','FontSize',12);
title('Analysis of the error (m/s) incurred by radar','FontSize',12);
grid minor;
subplot(2,1,2); hold on;
plot(rNPKTList,t_radar.*1e-3,'k-','LineWidth',2);
tovmax = max(t_radar.*1e-3);
nSlotTot = ceil(tovmax/TSLOT);
for idx = 1:nSlotTot
    plot(rNPKTList,(idx*TSLOT).*ones(size(rNPKTList)),'Color',[155 155 155]./255,'LineStyle',':','LineWidth',2);
end
xlabel('Number of Packets','FontSize',12);
ylabel('Overhead (ms)','FontSize',12);
title('Analysis of the overhead (ms) incurred by radar','FontSize',12);
legend('Overhead (ms)','Slots');
grid minor;