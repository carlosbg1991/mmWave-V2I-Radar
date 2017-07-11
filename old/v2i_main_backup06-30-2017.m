clear; close all; clc;
% =========================== PARAMETERS ================================ %
% Communications parameters
TSLOT  = 32.767;      % Time slot of the 802.11ad in milliseconds (ms)
BANDWIDTH = 2.7e6;    % Available Bandwidth in the 60GHz band in Hz
BEAMWIDTH = 10;       % Beamwidth for 1 sector in degrees
TOT_COV_ANGLE = 120;  % Total Angle covered in degrees
TOT_COV_DIST = 200;   % Maximum communication range (radar and comms) in meters
TXPOWER = 20;         % Transmit power in dBm (typical from 0 to 20 dBm)
NOISEPOWER  = -95;    % Noise power in dBm (Typical from -90 to -105 dBm)
N_sectors = TOT_COV_ANGLE / BEAMWIDTH;  % Number of sectors
% Radar parameters
NPKTRadar = 10;   % Number of packets used for radar operations
BEAMRadar = 0.5;  % Beamwidth required by the radar in degrees
% Simulation parameters
NCARS   = 10;            % Number of cars in the Simulation
AVSPEED = 100;           % Average speed of cars in Km/h
NLANE = 1;               % Number of leans in the highway
BSLOC = 5;               % Distance from the BS to the road in meters
LANELENGTH = 3;          % Lane width in meters
SIMTIME = 1e5;           % Simulation time in ms
carsID = (1:1:NCARS);    % Car ID's
NSIMSLOTS = ceil(SIMTIME/TSLOT);  % Slots over which the system simulates
% =========================== STATIC CONFIGURATION ====================== %
% Profile radar
[r_error,r_timeSect,r_slots] = v2i_conf_radar(NPKTRadar,BEAMRadar,BEAMWIDTH,TSLOT);
% Determine location of the BS
% To-Do
% Calculate sector limits
LANEID = 1;  % Lanes are given an id starting at 1 (closest to the BS)
[BSLocX,LocLaneY,sInt] = v2i_conf_sectors(BEAMWIDTH,TOT_COV_ANGLE,BSLOC,LANELENGTH,LANEID);
% % Generate initial time for each car to exit the simulation ROI
% cInitTime = 10000.*rand(NCARS,1); % Initi time uniform(0,10) seconds
% % Velocities uniformly distributed between minSpeed and maxSpeed (km/h)
% minSpeed = 5;
% maxSpeed = 120;
% cVel = minSpeed + (maxSpeed-minSpeed).*rand(NCARS,1);
% cVel = cVel./3600;        % Velocity in m/ms
% cLoc = -cVel.*cInitTime;  % Initial car location (always <= 0)
% New Initial car parameters calculation
minLoc = min(sInt);   % Closest car location to the ROI in meters
maxLoc = 2*min(sInt); % Fardest car location to the ROI in meters
cLoc = minLoc + (maxLoc-minLoc).*rand(NCARS,1);
minSpeed = 60;   % Minimum car velocity in Km/h
maxSpeed = 130;  % Maximum car velocity in Km/h
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
locCum = [];
Throughput = zeros(NCARS,NSIMSLOTS);
for nSlot = 1:NSIMSLOTS
	% Distances from Base Station
    for cID = carsID
        idxInt = find(tInt{cID} <= tSym, 1, 'last');
        loc(cID) = loc(cID) + vInt{cID}(idxInt)*tSym;
    end
    locCum = [locCum loc]
    if prod(loc > 0)
        fprintf('END OF EXECUTION - Cars out of the ROI\n');
        break;
    end
    distToBS = sqrt(LocLaneY^2 + loc.^2);
    % Calculate throughputs
    Throughput(:,nSlot) = v2i_capacities(distToBS,TXPOWER,NOISEPOWER,BANDWIDTH,TSLOT);
    % Update time to the next slot
    tSym = tSym + TSLOT;
end

ThRel = Throughput(:,1:nSlot-1);





