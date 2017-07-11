clear; close all; clc;
% =========================== PARAMETERS ================================ %
% Communications parameters
TSLOT  = 32.767/2;    % Time slot of the 802.11ad in milliseconds (ms)
BANDWIDTH = 2.7e6;    % Available Bandwidth in the 60GHz band in Hz
BEAMWIDTH = 3;        % Beamwidth for 1 sector in degrees
TOT_COV_ANGLE = 120;  % Total Angle covered in degrees
TOT_COV_DIST = 200;   % Maximum communication range (radar and comms) in meters
TXPOWER = 20;         % Transmit power in dBm (typical from 0 to 20 dBm)
NOISEPOWER  = -95;    % Noise power in dBm (Typical from -90 to -105 dBm)
N_sectors = TOT_COV_ANGLE / BEAMWIDTH;  % Number of sectors
% Radar parameters
rNPKT = 10;      % Number of packets used for radar operations
rBEAM = 0.5;     % Beamwidth required by the radar in degrees
rPATT = [ 1 0 0 1 0 0 0 ...
          6 0 0 6 0 0 0 ];  % Pattern that radar will follow in time. 
                            % The number represents the spatial sector. 
                            % 0 represents no radar operations

% Simulation parameters
NCARS   = 10;            % Number of cars in the Simulation
AVSPEED = 100;           % Average speed of cars in Km/h
NLANE = 1;               % Number of leans in the highway
BSLOC = 5;               % Distance from the BS to the road in meters
LANELENGTH = 3;          % Lane width in meters
SIMTIME = 1e5;           % Simulation time in ms
carsID = (1:1:NCARS);    % Car ID's
NSIMSLOTS = ceil(SIMTIME/TSLOT);  % Slots over which the system simulates
rPATT = repmat(rPATT,1,ceil(NSIMSLOTS/length(rPATT)));
% Scheduling parameters
POLICY = 'random';  % Scheduling policy for every slot ('greedy' or 'random')
% =========================== STATIC CONFIGURATION ====================== %
% Profile radar
[r_error,r_timeSect,r_slots] = v2i_conf_radar(rNPKT,rBEAM,BEAMWIDTH,TSLOT);
% Determine location of the BS
% To-Do
% Calculate sector limits
LANEID = 6;  % Lanes are given an id starting at 1 (closest to the BS)
[BSLocX,LocLaneY,sInt] = v2i_conf_sectors(BEAMWIDTH,TOT_COV_ANGLE,BSLOC,LANELENGTH,LANEID);
% % Generate initial time for each car to exit the simulation ROI
minLoc = min(sInt);   % Closest car location to the ROI in meters
maxLoc = 2*min(sInt); % Fardest car location to the ROI in meters
cLoc = minLoc + (maxLoc-minLoc).*rand(NCARS,1);
minSpeed = 50;   % Minimum car velocity in Km/h
maxSpeed = 80;   % Maximum car velocity in Km/h
cVel = minSpeed + (maxSpeed-minSpeed).*rand(NCARS,1); % Velocity in km/h
cVel = cVel./3600;  % Velocity in m/ms
cInitTime = - cLoc ./ cVel;  % Time to reach the end of the ROI in ms
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
locTot = zeros(NCARS,NSIMSLOTS);
distToBS = zeros(NCARS,NSIMSLOTS);
SNRTot = zeros(NCARS,NSIMSLOTS);
ThTot = zeros(NCARS,NSIMSLOTS);
sched = zeros(NSIMSLOTS);
vEst = cell(NCARS,1);
for nSlot = 1:NSIMSLOTS
	% Distances from Base Station
    for cID = carsID
        idxInt = find(tInt{cID} <= tSym, 1, 'last');
        loc(cID) = loc(cID) + vInt{cID}(idxInt)*tSym;
    end
    % End execution if all cars have exited the ROI
    if prod(loc > 0)
        fprintf('END OF EXECUTION - Cars out of the ROI\n');
        break;
    end
    % BS located at (x=BSLocX,y=0). Cars located at (x=loc,y=LocLaneY)
    distToBS(:,nSlot) = sqrt(LocLaneY^2 + (BSLocX - loc).^2);
    % Calculate throughputs
    [SNRTot(:,nSlot) , ThList] = ...
        v2i_capacities(distToBS(:,nSlot),TXPOWER,NOISEPOWER,BANDWIDTH,TSLOT);
    locTot(:,nSlot) = loc;
    % Radar operations based on pattern
    % To-Do
    % Schedule car based on policy
    sched(nSlot) = v2i_scheduling(sInt,loc,ThTot(:,nSlot),POLICY);
    if sched(nSlot) ~= 0 
        ThTot(sched(nSlot),nSlot) = ThList(sched(nSlot));
    end
    % Update time to the next slot
    tSym = tSym + TSLOT;
end

SNRCut = SNRTot(:,1:nSlot-1);
ThCut = sum(ThTot(:,1:nSlot-1));
locCut = locTot(:,1:nSlot-1);
distToBSCut = distToBS(:,1:nSlot-1);
schedCut = sched(1:nSlot-1);

fprintf('============= REPORT =============\n');
fprintf('Average Throughput scheduled %.2f Mbps (Shannon bound)\n',mean(ThCut).*1e-3);
fprintf('Jain-Fairness index %.2f\n',(sum(ThCut)^2)/(NCARS*sum(ThCut.^2)));

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
bar(ThCut);

% To-Do List
% (1) Start thinking about the functioning of the radar. We have to define 2
% things:
% - time/periodicity: How many % of the total slots are we willing to spend
% on radar operations
% - Location: Which sectors are we going to designate for the radar.
% - How to cobine these two parameters in time. For instance, for 50% radar
% and sectors 1 and 6 we need to define the sequences:
% seq1: [1 1 6 6]
% seq2: [1 6 1 6]
% seq3: [6 6 1 1]
% seq2: [6 1 6 1]
% where each index in the seqID vector represents the sector to which
% perform radar operations at time slot idx.
% (2) Incorporate velocity estimations
% (3) Incorporate function that checks wheather the transmission was
% succesful or failed based on the predicted location of the car.