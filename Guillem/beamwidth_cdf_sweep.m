clear all;
close all;

t=0:0.1:15;
%beamwidth=2.5;
avg_velocity=110;
std_deviation_velocity=10;
dist_max=150;
ang_max=120;
number_of_samples=10000;
doppler_res=2;

%% Calculate instant and sector loss for different beamwidths
[ t10, s10] = misalignment_sectortime( 1, dist_max, ang_max, avg_velocity, std_deviation_velocity, number_of_samples, doppler_res,t);
[ t20, s20] = misalignment_sectortime( 2, dist_max, ang_max, avg_velocity, std_deviation_velocity, number_of_samples, doppler_res,t);
[ t25, s25] = misalignment_sectortime( 2.5, dist_max, ang_max, avg_velocity, std_deviation_velocity, number_of_samples, doppler_res,t);
[ t30, s30] = misalignment_sectortime( 3, dist_max, ang_max, avg_velocity, std_deviation_velocity, number_of_samples, doppler_res,t);

%% Calculate de CDF for each one.

[f10,x10]=ecdf(t(t10));
[f20,x20]=ecdf(t(t20));
[f25,x25]=ecdf(t(t25));
[f30,x30]=ecdf(t(t30));

%last position is removed, is easy to present the plot later 
x10(end)=[];
x20(end)=[];
x25(end)=[];
x30(end)=[];
f10(end)=[];
f20(end)=[];
f25(end)=[];
f30(end)=[];

%% Plots

figure,
subplot(2,2,1)
hold on;
plot(x10,f10);
title('CDF for 1º beamwidth')
xlabel('seconds')
hold off;

subplot(2,2,2)
hold on;
plot(x20,f20);
title('CDF for 2º beamwidth')
xlabel('seconds')
hold off;

subplot(2,2,3)
hold on;
plot(x25,f25);
title('CDF for 2.5º beamwidth')
xlabel('seconds')
hold off;

subplot(2,2,4)
hold on;
plot(x30,f30);
title('CDF for 3º beamwidth')
xlabel('seconds')
hold off;

figure,
plot(x10,f10);
hold on;
plot(x20,f20);
plot(x25,f25);
plot(x30,f30);
legend('beamwidth=1º','beamwidth=2º','beamwidth=2.5º','beamwidth=3º');
xlabel('seconds')
title('Misalignment Cumulative Distribution Function');
hold off;

avg_misalign_t10=mean(t(t10));
fprintf(1,'Misalingnment happens after %g seconds (average)  \n', avg_misalign_t10)

avg_misalign_t20=mean(t(t20));
fprintf(1,'Misalingnment happens after %g seconds (average)  \n', avg_misalign_t20)

avg_misalign_t25=mean(t(t25));
fprintf(1,'Misalingnment happens after %g seconds (average)  \n', avg_misalign_t25)

avg_misalign_t30=mean(t(t30));
fprintf(1,'Misalingnment happens after %g seconds (average)  \n', avg_misalign_t30)
