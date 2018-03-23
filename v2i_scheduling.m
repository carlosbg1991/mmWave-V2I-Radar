% This function allocates the pertinent car to the actual slot given a
% scheduling policy defined beforehand. The omnipotent system can only
% schedule cars if they are within the ROI. The real system can only
% schedule cars if they are within the ROI and were detected previously.
% Input parameters
% - To-Do
% Output parameters
% - To-Do
function [schedOmn,ThTotOmn,schedSys,ThTotSys] = v2i_scheduling(sInt,loc,locSys,BSLocX,LocLaneY,schedOmn,ThTotOmn,schedSys,ThTotSys,nSlot,conf)
    % Output initialization. If no car is scheduled, we set them to 0
    cID = 0;  cIDSys = 0;
    % Compute boundaries of the ROI.
    inferiorX = min(sInt);
    superiorX = max(sInt);
    cIDCand = find(((loc>=inferiorX).*(loc<=superiorX)) == 1);
    if ~isempty(cIDCand)
        % Compute the distances to all the cars (real)
        distToBS(:) = sqrt(LocLaneY^2 + (BSLocX - loc).^2);
        % Compute the SNR that every car would perceive it it was allocated
        [SNRList,THPTList] = v2i_capacities(distToBS,conf);
%         [SNRList,THPTList] = v2i_throughput(distToBS,conf);
%         [SNRList,THPTList] = v2i_throughput(distToBS,TXBEAMWIDTH_AZ,TXBEAMWIDTH_EL,RXBEAMWIDTH_AZ,RXBEAMWIDTH_EL,TXPOWER,BANDWIDTH);
        switch conf.POLICY
            case 'GREEDY';  cID = GREEDY_scheduling(cIDCand,SNRList);
            case 'RA';      cID = RA_scheduling(cIDCand);
            case 'RR';      cID = RR_scheduling(cIDCand,schedOmn,nSlot);
            case 'EDF';     cID = EDF_scheduling(cIDCand);
            case 'LLF';     cID = LLF_scheduling(cIDCand);
            case 'PF';      cID = PF_scheduling(cIDCand,SNRList,schedOmn,ThTotOmn);
            case 'NOVEL';   cID = NOVEL_scheduling(sInt,loc);
        end
        schedOmn(nSlot) = cID;
        % Compute throughput percieved for scheduled car - Omnipotent
%         if schedOmn(nSlot) ~= 0;   ThTotOmn(schedOmn(nSlot),nSlot) = THPTList(cID); end
        if schedOmn(nSlot) ~= 0;   ThTotOmn(nSlot,1) = THPTList(cID); end
    end
    % Compute the estimated distance to the cars the system has previously
    % detected. The initial location of the cars is set to infinite
    % originally
    cIDCandEst = find(locSys~=inf);
    if ~isempty(cIDCandEst)
        switch conf.POLICY
            case 'GREEDY';  cIDSys = GREEDY_scheduling(cIDCandEst,SNRList);
            case 'RA';      cIDSys = RA_scheduling(cIDCand);
            case 'RR';      cIDSys = RR_scheduling(cIDCand,schedSys,nSlot);
            case 'EDF';     cIDSys = EDF_scheduling(cIDCand);
            case 'LLF';     cIDSys = LLF_scheduling(cIDCand);
            case 'PF';      cIDSys = PF_scheduling(cIDCand,SNRList,schedSys,ThTotSys);
            case 'NOVEL';   cIDSys = NOVEL_scheduling(sInt,locSys);
        end
        schedSys(nSlot) = cIDSys;
        % Compute throughput percieved for scheduled car - System
%         if schedSys(nSlot) ~= 0;   ThTotSys(schedSys(nSlot),nSlot) = THPTList(cIDSys); end
        if schedSys(nSlot) ~= 0;   ThTotSys(nSlot,1) = THPTList(cIDSys); end
    end
end

% Greedy Scheduling dependent on the SNR
function cID = GREEDY_scheduling(cIDCand,SNR)
    SNRFeas = SNR(cIDCand);
    [~,idx] = max(SNRFeas);
    cID = cIDCand(idx);
end

% Random Scheduling
function cID = RA_scheduling(cIDCand)
    idx = randi([1 length(cIDCand)]);
    cID = cIDCand(idx);
end

% Round Robin Scheduling
function cID = RR_scheduling(candList,schedList,nSlot)
    idxList = find(schedList~=0);
    sched = schedList(idxList);
    for k = 1:length(candList)
        if ~ismember(candList(k),sched)
            cID = candList(k);
            return;
        end
    end
    candListNew = circshift(candList,nSlot-idxList(1));
    cID = candListNew(1);
end

% Proporcional Fairness Scheduling
% [1] Yang Ji; Zhang Yifan; Wang Ying; Zhang Ping (2004-11-29), "Average
% rate updating mechanism in proportional fair scheduler for HDR", IEEE
% Global Telecommunications Conference, 2004, 6, IEEE, pp. 3464?3466,
% doi:10.1109/GLOCOM.2004.1379010, ISBN 0-7803-8794-5
% To-do: It runs extremely slow do to the huge data transfer between
% functions. Devise a plan to reduce the overhead and fasten the execution
function cID = PF_scheduling(cIDCand,SNR,schedList,ThTot)
    alpha = 1;
    beta = 1;
    P = zeros(length(cIDCand),1);
    for idx = 1:length(cIDCand)
        cand = cIDCand(idx);
        T = ( SNR(cand) )^alpha;
        R = ( mean(ThTot(schedList==cand,1)) )^beta;
        if isnan(R); R = 0; end
        P(idx) = abs(T / R);
    end
    [~,idxMax] = max(P);
    cID = cIDCand(idxMax);
end

% Earliest Deadline First Scheduling
function cID = EDF_scheduling(NCARS)
    % To-Do
end

% Least Laxity First Scheduling
function cID = LLF_scheduling(NCARS)
    % To-Do
end

function cID = NOVEL_scheduling(NCARS)
    % To-Do
end