clear; close all; clc;
% =========================== PARAMETERS ================================ %
% COMM PARAMETERS
TSLOT  = 32.767/20;   % Time slot of the 802.11ad in milliseconds (ms)
BANDWIDTH = 2.16e6;   % Available Bandwidth in the 60GHz band in Hz
BEAMWIDTH = 3;        % Beamwidth for 1 sector in degrees
TXBEAMWIDTH_AZ = 3;   % Azmiuth TX beamwidth for 1 sector in degrees
TXBEAMWIDTH_EL = [];  % Elevation TX beamwidth in degrees
RXBEAMWIDTH_AZ = TXBEAMWIDTH_AZ;  % Azimuth RX beamwidth in degrees
RXBEAMWIDTH_EL = [];  % Elevation RX beamwidth in degrees
TOT_COV_ANGLE = 120;  % Total Angle covered in degrees
TOT_COV_DIST = 200;   % Maximum communication range (radar and comms) in meters
TXPOWER = 20;         % Transmit power in dBm (typical from 0 to 20 dBm)
NOISEPOWER  = -95;    % Noise power in dBm (Typical from -90 to -105 dBm)
N_sectors = TOT_COV_ANGLE / BEAMWIDTH;  % Number of sectors
% RADAR PARAMETERS
rNPKT = 50;      % Number of packets used for radar operations
rBEAM = 0.5;     % Beamwidth required by the radar in degrees
% SIMULATION PARAMETERS
NCARS = 10;              % Number of cars in the Simulation
NLANE = 1;               % Number of leans in the highway
BSLOC = 5;               % Distance from the BS to the road in meters
LANELENGTH = 3;          % Lane width in meters
SIMTIME = 1e5;           % Simulation time in ms
carsID = (1:1:NCARS);    % Car ID's
NSIMSLOTS = ceil(SIMTIME/TSLOT);  % Slots over which the system simulates
% SCHEDULING PARAMETERS
POLICY = 'random';  % Scheduling policy for every slot ('greedy' or 'random')
% =========================== STATIC CONFIGURATION ====================== %
% SECTOR LIMITS
LANEID = 6;  % Lanes are given an id starting at 1 (closest to the BS)
[BSLocX,LocLaneY,sInt] = v2i_conf_sectors(BEAMWIDTH,TOT_COV_ANGLE,BSLOC,LANELENGTH,LANEID);
% GENERATE INITIAL VELOCITIES AND LOCATIONS
minLoc = min(sInt);   % Closest car location to the ROI in meters
maxLoc = 2*min(sInt); % Fardest car location to the ROI in meters
cLoc = minLoc + (maxLoc-minLoc).*rand(NCARS,1);
minSpeed = 50;   % Minimum car velocity in Km/h
maxSpeed = 80;   % Maximum car velocity in Km/h
cVel = minSpeed + (maxSpeed-minSpeed).*rand(NCARS,1); % Velocity in km/h
cVel = cVel./3600;  % Velocity in m/ms
cInitTime = - cLoc ./ cVel;  % Time to reach the end of the ROI in ms
% OPTIMUM RADAR CONFIGURATION - see ICC paper for details
Tradarmin = ceil(min(diff(sInt))/max(cVel));  % Radar MIN Periodicity
Tradarmin = ceil(Tradarmin/TSLOT);  % Radar Periodicity in terms of slots
rPATT = [1 1 zeros(1,Tradarmin) ];   % Pattern that radar will follow in time.
                                   % The number represents the spatial sector.
                                   % 0 represents no radar operations
rPATT = repmat(rPATT,1,ceil(NSIMSLOTS/length(rPATT)));  % Cover whole sim time.
% PROFILE RADAR
[e_radar,t_radar,s_radar] = v2i_conf_radar(maxSpeed,rNPKT,rBEAM,BEAMWIDTH,TSLOT);
% =========================== CORRECTIONS =============================== %
% Collision detection and velocity correction
[xInt,vInt,tInt] = v2i_collision_correction(cInitTime,cVel,cLoc,NCARS);
% =========================== SIMULATION ================================ %
% Main Simulation. We go over each Time Slot and perform simple operations
% such as (1) calculate the distance from the Base Station (BS), (2)
% calculate the SNR that each car would get and use it to (3) calculate the
% Capacity in Mbps.
tSym = 0;
loc = cLoc;  % Location of the cars. It is updated at each tSym
locTot = zeros(NCARS,NSIMSLOTS);  % Summary of locations of all the cars
locTot(:,1) = loc;  % Initial location summary
distToBS = zeros(NCARS,NSIMSLOTS);  % Summary of the distances
SNRTot = zeros(NCARS,NSIMSLOTS);  % Summary of the SNR of every car
ThTotOmn = zeros(NCARS,NSIMSLOTS);  % Summary oif the throughput (Omnipotent)
ThTotSys = zeros(NCARS,NSIMSLOTS);  % Summary oif the throughput (System)
schedOmn = zeros(NSIMSLOTS,1);  % Car scheduled at each slot (Omnipotent)
schedSys = zeros(NSIMSLOTS,1);  % Car scheduled at each slot (System)
vSys = cell(NCARS,1);  % Estimation of the speed of cars
tSys = cell(NCARS,1);  % Time the estimation happened
locSys = inf(NCARS,1);  % Estimation of the location of cars
for nSlot = 1:NSIMSLOTS
    % End execution if all cars have exited the ROI
    if prod(loc > 0); fprintf('END OF EXECUTION - Cars out of the ROI\n'); break; end
    % BS located at (x=BSLocX,y=0). Cars located at (x=loc,y=LocLaneY)
    distToBS(:,nSlot) = sqrt(LocLaneY^2 + (BSLocX - loc).^2);
    % Calculate throughputs - it only serves for plotting
%     [SNRTot(:,nSlot),ThList] = v2i_capacities(distToBS(:,nSlot),TXPOWER,NOISEPOWER,BANDWIDTH,TSLOT);
    [SNRTot(:,nSlot),ThList] = v2i_throughput(distToBS(:,nSlot),TXBEAMWIDTH_AZ, TXBEAMWIDTH_EL, RXBEAMWIDTH_AZ, RXBEAMWIDTH_EL, TXPOWER, BANDWIDTH);
    % Decide wheather we need to perform radar operations
    if rPATT(nSlot) ~=0
        % Radar operations based on pattern
        [vSys,tSys,locSys] = v2i_radar_operations(nSlot,tSym,rPATT,loc,sInt,vInt,tInt,e_radar,vSys,tSys,locSys);
        % Radar operations may take longer than one slot. The system needs
        % to increase the simulation time accordingly
        kmul = s_radar;
    else
        % Schedule car based on policy
        [schedOmn(nSlot),ThTotOmn1,schedSys(nSlot),ThTotSys1] = v2i_scheduling(sInt,loc,locSys,BSLocX,LocLaneY,TXPOWER,NOISEPOWER,BANDWIDTH,TSLOT,POLICY);
        % Compute throughput percieved for scheduled car - Omnipotent
        if schedOmn(nSlot) ~= 0;   ThTotOmn(schedOmn(nSlot),nSlot) = ThTotOmn1; end
        % Compute throughput percieved for scheduled car - System
        if schedSys(nSlot) ~= 0;   ThTotSys(schedSys(nSlot),nSlot) = ThTotSys1; end
        % Communications take only 1 slot per iteration -> kmul=1
        kmul = 1;
    end
    % Update time to the next slot
	tSym = tSym + kmul*TSLOT;
    % Update locations loc (Omnipotent) and locEst (System)
    [loc,locSys] = v2i_update_location(loc,locSys,vSys,tSys,carsID,tInt,vInt,s_radar,tSym,TSLOT,kmul);
    % Update summary of locations
    locTot(:,nSlot) = loc;
end

SNRCut = SNRTot(:,1:nSlot-1);
ThCutOmn = sum(ThTotOmn(:,1:nSlot-1));
ThCutSys = sum(ThTotSys(:,1:nSlot-1));
locCut = locTot(:,1:nSlot-1);
distToBSCut = distToBS(:,1:nSlot-1);
schedCut = schedOmn(1:nSlot-1);

fprintf('============= REPORT =============\n');
fprintf('(Omnipotent) - Average Throughput %.2f Mbps (Shannon bound)\n',mean(ThCutOmn).*1e-3);
fprintf('(Omnipotent) - Jain-Fairness index %.2f\n',(sum(ThCutOmn)^2)/(NCARS*sum(ThCutOmn.^2)));
fprintf('(System)     - Average Throughput %.2f Mbps (Shannon bound)\n',mean(ThCutSys).*1e-3);
fprintf('(System)     - Jain-Fairness index %.2f\n',(sum(ThCutSys)^2)/(NCARS*sum(ThCutSys.^2)));
fprintf('Overhead Radar = %.3f (%)\n',100*(length(find(rPATT~=0))/length(rPATT)));

% =========================== PLOTTING ================================== %
LineStyle     = {':','-','-.','--'};
ColorList     = {'c','b','m','k'};
leg = cell(NCARS,1);
figure(1); hold on;
figure(2); hold on;
for id = 1:NCARS
    ix1  = mod(id,length(LineStyle)) + 1;
    ix2  = mod(ceil(id/length(LineStyle)),length(ColorList)) +  1;
    figure(1);
    plot((1:nSlot-1),SNRCut(id,:),'Color',ColorList{ix2},...
        'Linestyle',LineStyle{ix1},'LineWidth',2);
    figure(2);
    plot((1:nSlot-1),distToBSCut(id,:),'Color',ColorList{ix2},...
        'Linestyle',LineStyle{ix1},'LineWidth',2);
    leg{id} = strcat('Car with ID=',strtrim(num2mstr(id)));
end
figure(1);
legend(leg);
xlabel('Slots')
title('SNR');
grid minor;
figure(2);
dMaxInROI = sqrt( LocLaneY.^2 + (BSLocX-sInt(1)).^2 );
plot((1:nSlot-1),dMaxInROI.*ones(nSlot-1,1),'k-');
legend(leg);
title('Distance to BS');
grid minor;

figure(3);
subplot(1,2,1);
bar(ThCutOmn);
subplot(1,2,2);
bar(ThCutSys);

% To-Do List
% (END) Last requirement to be implemented. Incorporate function that
% checks wheather the transmission was succesful or failed based on the
% predicted location of the car. KEY