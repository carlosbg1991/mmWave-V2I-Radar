function [schedOmn,ThCutOmn,schedSys,ThCutSys] = v2i_main1(conf,BSLocX,LocLaneY,sInt,cLoc,cVel,cInitTime,e_radar,s_radar,rPATT,figIdx,DEBUG)
% =========================== CORRECTIONS =============================== %
% Collision detection and velocity correction
[~,vInt,tInt] = v2i_collision_correction(cInitTime,cVel,cLoc,conf.NCARS,DEBUG);
% =========================== SIMULATION ================================ %
% Main Simulation. We go over each Time Slot and perform simple operations
% such as (1) calculate the distance from the Base Station (BS), (2)
% calculate the SNR that each car would get and use it to (3) calculate the
% Capacity in Mbps.
tSym = 0;
loc = cLoc;  % Location of the cars. It is updated at each tSym
locTot = zeros(conf.NCARS,conf.NSIMSLOTS);  % Summary of locations of all the cars
locTot(:,1) = loc;  % Initial location summary
distToBS = zeros(conf.NCARS,conf.NSIMSLOTS);  % Summary of the distances
SNRTot = zeros(conf.NCARS,conf.NSIMSLOTS);  % Summary of the SNR of every car
ThTotOmn = zeros(conf.NSIMSLOTS,1);  % Summary oif the throughput (Omnipotent)
ThTotSys = zeros(conf.NSIMSLOTS,1);  % Summary oif the throughput (System)
schedOmn = zeros(conf.NSIMSLOTS,1);  % Car scheduled at each slot (Omnipotent)
schedSys = zeros(conf.NSIMSLOTS,1);  % Car scheduled at each slot (System)
vSys = cell(conf.NCARS,1);  % Estimation of the speed of cars
tSys = cell(conf.NCARS,1);  % Time the estimation happened
locSys = inf(conf.NCARS,1);  % Estimation of the location of cars
for nSlot = 1:conf.NSIMSLOTS
    % End execution if all cars have exited the ROI
    if prod(loc > 0); if DEBUG; fprintf('END OF EXECUTION - Cars out of the ROI\n'); end; break; end
    % BS located at (x=BSLocX,y=0). Cars located at (x=loc,y=LocLaneY)
    distToBS(:,nSlot) = sqrt(LocLaneY^2 + (BSLocX - loc).^2);
    % Calculate throughputs - it only serves for plotting
    [SNRTot(:,nSlot) , ~] = v2i_capacities(distToBS(:,nSlot),conf);
%     [SNRTot(:,nSlot), ~] = v2i_throughput(distToBS(:,nSlot),conf);
    % Decide wheather we need to perform radar operations
    if rPATT(nSlot) ~=0
        % Radar operations based on pattern
        [vSys,tSys,locSys] = v2i_radar_operations(nSlot,tSym,rPATT,loc,sInt,vInt,tInt,e_radar,vSys,tSys,locSys,DEBUG);
        % Radar operations may take longer than one slot. The system needs
        % to increase the simulation time accordingly
        kmul = s_radar;
    else
        % Schedule car based on policy
        [schedOmn,ThTotOmn,schedSys,ThTotSys] = v2i_scheduling(sInt,loc,locSys,BSLocX,LocLaneY,schedOmn,ThTotOmn,schedSys,ThTotSys,nSlot,conf);
        % Communications take only 1 slot per iteration -> kmul=1
        kmul = 1;
    end
    % Update time to the next slot
	tSym = tSym + kmul*conf.TSLOT;
    % Update locations loc (Omnipotent) and locEst (System)
    [loc,locSys] = v2i_update_location(loc,locSys,vSys,tSys,conf.CARSID,tInt,vInt,s_radar,tSym,conf.TSLOT,kmul);
    % Update summary of locations
    locTot(:,nSlot) = loc;
end

SNRCut = SNRTot(:,1:nSlot-1);
ThCutOmn = ThTotOmn(1:nSlot-1,1);
ThCutSys = ThTotSys(1:nSlot-1,1);
distToBSCut = distToBS(:,1:nSlot-1);
schedOmnCut = schedOmn(1:nSlot-1);
schedSysCut = schedSys(1:nSlot-1);

if DEBUG
    fprintf('============= REPORT =============\n');
    fprintf('(Omnipotent) - Average Throughput %.2f Mbps (Shannon bound)\n',mean(ThCutOmn).*1e-3);
    fprintf('(Omnipotent) - Jain-Fairness index %.2f\n',(sum(ThCutOmn)^2)/(conf.NCARS*sum(ThCutOmn.^2)));
    fprintf('(System)     - Average Throughput %.2f Mbps (Shannon bound)\n',mean(ThCutSys).*1e-3);
    fprintf('(System)     - Jain-Fairness index %.2f\n',(sum(ThCutSys)^2)/(conf.NCARS*sum(ThCutSys.^2)));
    fprintf('Overhead Radar = %.3f (%%)\n',100*(length(find(rPATT~=0))/length(rPATT)));
end

% =========================== PLOTTING ================================== %
LineStyle     = {':','-','-.','--'};
ColorList     = {'c','b','m','k'};
leg = cell(conf.NCARS,1);
figure(figIdx); hold on;
figure(figIdx+1); hold on;
for id = 1:conf.NCARS
    ix1  = mod(id,length(LineStyle)) + 1;
    ix2  = mod(ceil(id/length(LineStyle)),length(ColorList)) +  1;
    figure(figIdx); hold on;
    plot((1:nSlot-1),SNRCut(id,:),'Color',ColorList{ix2},...
        'Linestyle',LineStyle{ix1},'LineWidth',2);
    figure(figIdx+1); hold on;
    plot((1:nSlot-1),distToBSCut(id,:),'Color',ColorList{ix2},...
        'Linestyle',LineStyle{ix1},'LineWidth',2);
    leg{id} = strcat('Car with ID=',strtrim(num2mstr(id)));
end
figure(figIdx);
legend(leg);
xlabel('Slots')
title('SNR');
grid minor;
figure(figIdx+1);
dMaxInROI = sqrt( LocLaneY.^2 + (BSLocX-sInt(1)).^2 );
plot((1:nSlot-1),dMaxInROI.*ones(nSlot-1,1),'k-');
legend(leg);
title('Distance to BS');
grid minor;

figure(figIdx+2);
subplot(1,2,1);
b1 = bar(ThCutOmn.*1e-3);
b1.FaceColor = 'flat';
for n = conf.CARSID
    idx = find(schedOmnCut==n);
    b1.CData(idx,:) = repmat(conf.colorList{n},length(idx),1);
end
xlabel('Slot index','FontSize',12);
ylabel('Throughput in Mbps','FontSize',12);
title('Omnipotent','FontSize',13);
grid minor;
subplot(1,2,2);
b2 = bar(ThCutSys.*1e-3);
b2.FaceColor = 'flat';
for n = conf.CARSID
    idx = find(schedSysCut==n);
    b2.CData(idx,:) = repmat(conf.colorList{n},length(idx),1);
end
xlabel('Slot index','FontSize',12);
ylabel('Throughput in Mbps','FontSize',12);
title('System','FontSize',13);
grid minor;
pos = get(gcf, 'Position');
locx = pos(3);
locy = pos(4);
set(gcf,'units','points','position',[locx,locy,400,200])

% To-Do List
% (END) Last requirement to be implemented. Incorporate function that
% checks wheather the transmission was succesful or failed based on the
% predicted location of the car. KEY