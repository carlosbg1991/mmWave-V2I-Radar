function [SNR, Ths] = v2i_throughput(DISTS, TXBEAMWIDTH_AZ, TXBEAMWIDTH_EL, RXBEAMWIDTH_AZ, RXBEAMWIDTH_EL ,TXPOWER ,BANDWIDTH)
%This function returns the maximum achievable throughput for the given
%inputs, being distance and beamwidth the design parameters. To do so, some
%simulations were performed in order to characterize the PERvsSNR. 
%For MCS 0, the PER shall be less than 5% for a PSDU length of 256 octets.
%For the other MCSs, the PER shall be less than 1% for a PSDU length of
%4096 octets. The function checks for which MCS does the received SNR
%fullfils the PER requirements and returns as an output the highest
%throughput that can be achieved. The mapping between throughput and each
%MCS has been taken from the standard 802.11ad.

load('PERvsSNR.mat') %loads the variable"PER_SNR", this matrix contains the PER
%of each MCS (each row index corresponds to the MCS excepts the 25th,
%which corresponds to MCS 0) in terms of the SNR, which can be found in the last row (26)

load('throughput.mat') %loads "throughput", this vector contains the throughput
%each MCS can achieve. Each vector index corresponds to the each MCS,
%except for the 25th, which corresponds to MCS 0.
n=2.66; %path-loss exponent
AtmAtt=0.015; %Atmospheric attenuation dBm/m
RainAtt=0.025; %Rain attenuation dBm/m
Chatt=70; %Channel attenuation (Constant)
NF=6; %Noise figure 
Nfloor=-174;
min_PER=0.01;  %min acceptable PER for every MCS but 0
min_PER_CONTROL=0.05; %acceptable PER for MCS 0


%% Random shadowing effect calculation lognormal(0,5.8^2)


%SF = random('Normal', 0, 5.8, 1, 1);
SF=0;

%% SNR calculation


Aatt=AtmAtt*DISTS;
Ratt=RainAtt*DISTS;
PL = 10*n*log10(DISTS) + SF + Chatt + Aatt + Ratt;
Gtx=4*(180^2)/(TXBEAMWIDTH_AZ*TXBEAMWIDTH_EL*pi);
Grx=4*(180^2)/(RXBEAMWIDTH_AZ*RXBEAMWIDTH_EL*pi);

NOISEPOWER = Nfloor + 10*log10(BANDWIDTH) + NF;

RXPOWER = TXPOWER + 10*log10(Gtx) - PL + 10*log10(Grx);

SNR = RXPOWER - NOISEPOWER;

%% Find the PER for each MCS in terms of SNR 


i_mins=[];
for i=1:length(SNR)
    [min_dist, i_min]=min(dist(SNR(i),PER_SNR(26,:)));
    if PER_SNR(26,i_min)>SNR(i) && i_min ~=1
        i_min=i_min-1;
    end
    i_mins=[i_mins;i_min];
end
    

PERs = PER_SNR(1:25,i_mins);

%% Find the feasible MCS and calculate the maximum throughput among the feasible ones

feas_mcs= diag(1:25)*[PERs(1:24,:)<min_PER; PERs(25,:)<min_PER_CONTROL];
m=feas_mcs~=0;
Ths=feas_mcs;
Ths(m)=throughput(feas_mcs(m));
Ths=max(Ths);



end

