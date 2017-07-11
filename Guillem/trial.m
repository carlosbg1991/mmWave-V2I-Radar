clear all;
close all;

%% Inputs
t                 = (0:0.1:15);    % Time vector in seconds
beamwidth         = 3;             % Beamwidth in degrees
avg_vel           = 110;           % Speed in Km/h
std_dev_vel       = 10;            % Gaussian distr with mean=avg_velocity and
                                   % std=std_deviation_velocity
dist_max          = 200;           % Max coverage distance
ang_max           = 120;           % Angle that covers the BS
number_of_samples = 1e5;           % Number of iterations
doppler_res       = 2;             % Doppler resolution

LineStyle     = {':','-','-.','--'};
ColorList     = {'c','b','m','k'};

%% Main Function
% misalignment_sectortime() returns the index of the vector t where
% misalignment happened (ts) and the sector where misalingment happended (s)
beamwidthList = [1 4 7 11];         % Beamwidth
speedList     = [30 50 70 90 110];  % Speed
ts       = zeros(number_of_samples,length(beamwidthList),length(speedList));
s        = zeros(number_of_samples,length(beamwidthList),length(speedList));
meanMiss = zeros(length(beamwidthList),length(speedList));
for idx1 = 1:length(beamwidthList)
    beamwidth = beamwidthList(idx1);
parfor idx2 = 1:length(speedList)
    avg_vel = speedList(idx2);
    [ts(:,idx1,idx2),s(:,idx1,idx2)] = misalignment_sectortime10( ...
                                        beamwidth, dist_max, ang_max, ...
                                        avg_vel, std_dev_vel, ...
                                        number_of_samples, doppler_res,t);
end
end

%% Results
% cdf Calculations
cnt = 1;
leg1 = cell(length(beamwidthList)*length(speedList),1);
leg2 = cell(length(beamwidthList),1);
for idx1 = 1:length(beamwidthList)
for idx2 = 1:length(speedList)
    meanMiss(idx1,idx2) = mean(t(ts(:,idx1,idx2)));
    fprintf('%beam %d - speed %d - Misaling. at %g (s)\n', ...
                           beamwidthList(idx1), speedList(idx2), meanMiss);
    
    [f,x]=ecdf(t(ts(:,idx1,idx2)));
    [fs,xs]=ecdf(s(:,idx1,idx2));

    x(end)=[];
    f(end)=[];
    
    ix1  = mod(idx1,length(LineStyle)) + 1; % Controls Color
    ix2  = mod(idx2,length(LineStyle)) + 1; % Controls LineStyle
    leg1{cnt} = strcat(num2str(beamwidthList(idx1)), 'deg',...
                            ' - ',num2str(speedList(idx2)),...
                            ' m/s');
	cnt = cnt + 1;

    figure(1); hold on;
    plot(x,f,LineStyle{ix2},'Color',ColorList{ix1},'LineWidth',2);
    xlabel("Time");
    ylabel("Cumulated probability");
    title("Misalignment Cumulated densitity probability in terms of time");
    grid minor;

    figure(2); hold on;
    plot(xs,fs,LineStyle{ix2},'Color',ColorList{ix1},'LineWidth',2);
    xlabel("Sector");
    ylabel("Cumulated probability");
    title("Misalignment Cumulated densitity probability in terms of sector");
    grid minor;
end
    figure(3); hold on
    plot(speedList,meanMiss(idx1,:),LineStyle{2},'Color',ColorList{ix1},...
                    'LineWidth',2,'Marker','square');
    xlabel("Speed (m/s)");
    ylabel("Average time to Missalignment (s)");
    title("Average time to Misalignment ");
    grid minor;
    
    leg2{idx1} = strcat('Beamwidth:', num2str(beamwidthList(idx1)), 'deg');
end

figure(1);
legend(leg1,'Location','NorthEast');
set(gca,'xscale','log');
grid minor;
figure(2);
legend(leg1,'Location','NorthEast');
set(gca,'xscale','log');
grid minor;
figure(3);
legend(leg2,'Location','NorthEast');
grid minor;