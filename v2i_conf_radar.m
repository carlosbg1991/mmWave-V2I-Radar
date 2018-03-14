% This function configures the accuracy and the overhead incurred by the
% radar. It returns 3 parameters:
% - e_radar: The doppler resolution in m/ms. The accuracy of the radar.
% - t_radar: The required time to perform radar operation within a sector.
% - s_radar: The number of slots that radar will take over consecutively 
function [e_radar,t_radar,s_radar] = v2i_conf_radar(maxVel,NPKTRadar,BEAMRadar,BEAMWIDTH,TSLOT)
    lambda = 3e8 / 60e9;     % Lambda operating at mmWaves
    T_HEADER = 1.6051*1e-3;  % Header time in ms
    T_DATA   = 7.2960*1e-3;  % Minimum Data time in ms
    T_AGC    = 2.918*1e-3;   % Automatic Gane Control in ms
    % Minimum PRI required to detect cars at maxVel (m/s)
    PRImin = fmaxVeltoPRI(lambda,maxVel);
    % Miminum Frame time accounting for the minimum required PRI
    T_FRAME  = PRImin + T_HEADER + T_DATA + T_AGC;
    % Doppler resolution (m/s)
    e_radar = faccuracyRadar(lambda,PRImin,NPKTRadar);
    % Radar time
    [t_radar,s_radar] = fRadarTime(T_FRAME,NPKTRadar,BEAMRadar,BEAMWIDTH,TSLOT);
end

% The function returns the minimum PRI as a function of the velocity the
% system needs to achieve (usually the maximum, to cover all cases)
function PRImin = fmaxVeltoPRI(lambda,vel)
    PRImin = (lambda./vel)*1e3;    % PRI minimum in ms
end

% The function returns the doppler resolution in m/s (error) that the radar
% gives us for a certain PRI and Number of packets.
function error = faccuracyRadar(lambda,PRI,NPKTRadar)
   CPI = PRI * NPKTRadar;  % PRI in (s) then CPI in (s)
   error = lambda./(2.*CPI);  % error in (m/ms)
end

function [t_radar,s_radar] = fRadarTime(T_FRAME,NPKTRadar,BEAMRadar,BEAMWIDTH,TSLOT)
    t_radar_uni = T_FRAME * NPKTRadar;  % Time required in each 0.5 degrees (ms)
    t_radar = (BEAMWIDTH/BEAMRadar) * t_radar_uni;  % Time required per sector (ms)
    s_radar = ceil(max(t_radar,TSLOT) / min(t_radar,TSLOT));  % Number of slots required for radar
end