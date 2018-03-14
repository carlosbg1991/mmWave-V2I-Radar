% Load saved figures
cGR=hgload('thDist_GREEDY.fig');
cRR=hgload('thDist_RR.fig');
cRA=hgload('thDist_RA.fig');
cPF=hgload('thDist_PF.fig');
% Prepare subplots
figure
h(1)=subplot(2,2,1);
h(2)=subplot(2,2,2);
h(3)=subplot(2,2,3);
h(4)=subplot(2,2,4);
% Paste figures on the subplots
copyobj(allchild(get(cGR,'CurrentAxes')),h(1));
copyobj(allchild(get(cRR,'CurrentAxes')),h(2));
copyobj(allchild(get(cRA,'CurrentAxes')),h(3));
copyobj(allchild(get(cPF,'CurrentAxes')),h(4));
% Add legends
l(1)=title(h(1),'Greedy scheduling','FontSize',12);
l(2)=title(h(2),'Round Robin scheduling','FontSize',12);
l(3)=title(h(3),'Random scheduling','FontSize',12);
l(4)=title(h(4),'Proportional Fairness scheduling','FontSize',12);