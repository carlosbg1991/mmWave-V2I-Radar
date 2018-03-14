function [conf,BSLocX,LocLaneY,sInt,cLoc,cVel,cInitTime,e_radar,t_radar,s_radar,rPATT] = v2i_config(varargin)
% MAIN PARAMETERS
conf.POLICY = 'GREEDY';   % Scheduling policy for every slot:'GREEDY' ,
                          % 'RA' , 'RR' , 'PF' , 'EDF' , 'LLF' or 'NOVEL'
conf.TSLOT  = 32.767/20;  % Time slot of the 802.11ad in milliseconds (ms)
conf.NCARS = 10;          % Number of cars in the Simulation
CONTROLLED = false;

if nargin==1;      conf.POLICY = varargin{1};
elseif nargin==2;  conf.POLICY = varargin{1};
                   conf.TSLOT  = varargin{2};
elseif nargin==3;  conf.POLICY = varargin{1};
                   conf.TSLOT  = varargin{2};
                   conf.NCARS  = varargin{3};
elseif nargin==4;  conf.POLICY = varargin{1};
                   conf.TSLOT  = varargin{2};
                   conf.NCARS  = varargin{3};
                   CONTROLLED  = varargin{4};
end
    
% COMM PARAMETERS
conf.BANDWIDTH = 2.16e6;   % Available Bandwidth in the 60GHz band in Hz
conf.BEAMWIDTH = 3;        % Beamwidth for 1 sector in degrees
conf.TXBEAMWIDTH_AZ = 3;   % Azmiuth TX beamwidth for 1 sector in degrees
conf.RXBEAMWIDTH_AZ = conf.TXBEAMWIDTH_AZ;  % Azimuth RX beamwidth in degrees
conf.TOT_COV_ANGLE = 120;  % Total Angle covered in degrees
conf.TOT_COV_DIST = 200;   % Maximum communication range (radar and comms) in meters
conf.TXPOWER = 20;         % Transmit power in dBm (typical from 0 to 20 dBm)
conf.NOISEPOWER = -95;     % Noise power in dBm (Typical from -90 to -105 dBm)
conf.N_sectors = conf.TOT_COV_ANGLE / conf.BEAMWIDTH;  % Number of sectors

% RADAR PARAMETERS
rNPKT = 50;      % Number of packets used for radar operations
rBEAM = 0.5;     % Beamwidth required by the radar in degrees

% SIMULATION PARAMETERS
conf.MINSPEED = 50;           % Minimum car velocity in Km/h
conf.MAXSPEED = 80;           % Maximum car velocity in Km/h
conf.NLANE = 1;               % Number of leans in the highway
conf.LANEID = 6;              % Lanes are given an id starting at 1 (closest to the BS)
conf.BSLOC = 5;               % Distance from the BS to the road in meters
conf.BSHEIGHT = 10;           % BS height in meters
conf.LANELENGTH = 3;          % Lane width in meters
conf.SIMTIME = 1e5;           % Simulation time in ms
conf.CARSID = (1:1:conf.NCARS);    % Car ID's
conf.NSIMSLOTS = ceil(conf.SIMTIME/conf.TSLOT);  % Slots over which the system simulates

% =========================== STATIC CONFIGURATION ====================== %
% SECTOR LIMITS
[BSLocX,LocLaneY,sInt] = v2i_conf_sectors(conf.TXBEAMWIDTH_AZ,conf.TOT_COV_ANGLE,conf.BSLOC,conf.LANELENGTH,conf.LANEID);

% ELEVATION ANGLES FOR TX AND RX
conf.TXBEAMWIDTH_EL = v2i_config_BSbeamwidthEL(conf.TOT_COV_ANGLE, LocLaneY, conf.BSHEIGHT, conf.LANEID);
conf.RXBEAMWIDTH_EL = conf.TXBEAMWIDTH_EL;

% GENERATE INITIAL VELOCITIES AND LOCATIONS
if ~CONTROLLED
 minLoc = min(sInt);   % Closest car location to the ROI in meters
 maxLoc = 2*min(sInt); % Fardest car location to the ROI in meters
 cLoc = minLoc + (maxLoc-minLoc).*rand(conf.NCARS,1);
 cVel = conf.MINSPEED + (conf.MAXSPEED-conf.MINSPEED).*rand(conf.NCARS,1); % Velocity in km/h
else
 load(strcat('controlled_',num2str(conf.NCARS)),'cLoc','cVel');
end
cVel = cVel./3600;  % Velocity in m/ms
cInitTime = - cLoc ./ cVel;  % Time to reach the end of the ROI in ms

% OPTIMUM RADAR CONFIGURATION - see ICC paper for details
Tradarmin = ceil(min(diff(sInt))/max(cVel));  % Radar MIN Periodicity
Tradarmin = ceil(Tradarmin/conf.TSLOT);  % Radar Periodicity in terms of slots
rPATT = [1 1 zeros(1,Tradarmin) ];  % Pattern that radar will follow in time.
                                    % The number represents the spatial sector.
                                    % 0 represents no radar operations
rPATT = repmat(rPATT,1,ceil(conf.NSIMSLOTS/length(rPATT)));  % Cover whole sim time.

% PROFILE RADAR
[e_radar,t_radar,s_radar] = v2i_conf_radar(conf.MAXSPEED,rNPKT,rBEAM,conf.TXBEAMWIDTH_AZ,conf.TSLOT);

% =========================== STATIC CONFIGURATION ====================== %
conf.colorList = {[255 51 51]./255, [255 128 0]./255, [128 255 0]./255,...
                  [51 255 255]./255, [0 128 255]./255, [0 0 255]./255,...
                  [127 0 255]./255, [255 0 255]./255, [255 0 127]./255,...
                  [102 0 0]./255, [102 51 0]./255, [102 102 0]./255,...
                  [51 102 0]./255, [0 51 102]./255, [0 0 102]./255,...
                  [51 0 102]./255, [102 0 102]./255, [102 0 51]./255};
end