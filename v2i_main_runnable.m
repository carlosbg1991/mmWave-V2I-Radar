clear; close all; clc
runnable = true;
% POLICYList = {'GREEDY','RA','RR','PF'};
POLICYList = {'PF'};
TSLOT = 32.767/20;
NCARS = 10;
DEBUG = false;
CONTROLLED = false;
for idx = 1:length(POLICYList)
% =========================== PARAMETERS ================================ %
POLICY = POLICYList{idx};
[conf,BSLocX,LocLaneY,sInt,cLoc,cVel,cInitTime,e_radar,t_radar,s_radar,rPATT] = v2i_config(POLICY,TSLOT,NCARS,CONTROLLED);
% =========================== MAIN FILE ================================= %
figIdx = (idx-1)*3 + 1;
[schedOmn,ThTotOmn,schedSys,ThTotSys] = v2i_main1(conf,BSLocX,LocLaneY,sInt,cLoc,cVel,cInitTime,e_radar,s_radar,rPATT,figIdx,DEBUG);
avThOmn = mean(ThTotOmn).*1e-3;
avThSys = mean(ThTotSys).*1e-3;
jainFairOmn = (sum(ThTotOmn)^2)/(NCARS*sum(ThTotOmn.^2));
jainFairSys = (sum(ThTotSys)^2)/(NCARS*sum(ThTotSys.^2));
fprintf('== REPORT POLICY %10s =============\n',POLICY);
fprintf('(Omnipotent) - Average Throughput %.2f Mbps (Shannon bound)\n',mean(ThTotOmn).*1e-3);
fprintf('(Omnipotent) - Jain-Fairness index %.2f\n',(sum(ThTotOmn)^2)/(conf.NCARS*sum(ThTotOmn.^2)));
fprintf('(System)     - Average Throughput %.2f Mbps (Shannon bound)\n',mean(ThTotSys).*1e-3);
fprintf('(System)     - Jain-Fairness index %.2f\n',(sum(ThTotOmn)^2)/(conf.NCARS*sum(ThTotOmn.^2)));
fprintf('Overhead Radar = %.3f (%%)\n',100*(length(find(rPATT~=0))/length(rPATT)));
end

% TO-DO list
% - Include car tracker at every slot - Pre-requisite error calculation.
% - Types of traffic and assign them to cars -> Demands in terms of
%   throughput and latency (Xavier Costa's paper - TABLE I).
% - Include error => Schedule a car and realize it's not within the estimated
%   sector.
% - Include Scheduling algorithms of RMS, EDF and LLF. First, understand
%   the differences between them.
% - Include scheduling algorithm that uses reinforcement learning to
%   improve the performance (novel).