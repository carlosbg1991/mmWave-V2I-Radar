function [SNR,Th] = v2i_capacities(distances,conf)
    TXPOWER = conf.TXPOWER;
    NOISEPOWER = conf.NOISEPOWER;
    BANDWIDTH = conf.BANDWIDTH;
    % Received Power Calculation [1]
    % Path loss model
    GTX = 10*log10(4*(180^2)/(3*360*pi));  % Antenna gain at the TX
    GRX = 10*log10(1);                     % Antenna gain at the RX
    n = 2.66;  % Shadowing factor
    AtmAtt = 0.015; % in dBm/m
    RainAtt = 0.025; % in dBm/m
    Chatt = 70;  % ??
    SF = 5;  % Shadow fading standard deviation
    d = abs(distances);
    
    Aatt = AtmAtt.*d;
    Ratt = RainAtt.*d;
    PL1  = 10.*n.*log10(d) + SF + Chatt + Aatt + Ratt;
    
    c = physconst('lightspeed');  % Propagation speed (m/s)
    fc = 60.48e9;                 % Center frequency (Hz)
    lambda = c/fc;
    PL2 =  10*n*log10(lambda./(4*pi*d)) - 10;

%     RXPOWER1 = TXPOWER + GTX + GRX + PL1;
    RXPOWER2 = TXPOWER + GTX + GRX + PL2;
%     RXPOWER = TXPOWER + PL2;
    % SNR Calculation
%     SNR = RXPOWER1 - NOISEPOWER;
    SNR = RXPOWER2 - NOISEPOWER;
    SNR_lin = 10.^(SNR./10);
    % Capacity following Shannon's bound
    Th = BANDWIDTH.*log2(1+SNR_lin);
    
    % Reference path loss model 
    % [1] https://arxiv.org/pdf/1511.07345.pdf
    % [2] V. Va, T. Shimizu, G. Bansal and R. W. Heath, "Beam design for 
    %     beam switching based millimeter wave vehicle-to-infrastructure 
    %     communications," 2016 IEEE ICC, Kuala Lumpur, 2016, pp. 1-6.
end