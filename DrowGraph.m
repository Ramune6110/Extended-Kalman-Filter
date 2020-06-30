clear;
close all;
clc;
% reading file
fileID = fopen('data.txt', 'r');
formatSpec = '%f';
size = [5 6000];
data = fscanf(fileID, formatSpec, size);
% graph
figure(1);
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');

figure(1);
%Plot Result
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(data(2, :), data(3, :),'-.b'); hold on;
plot(data(4, :), data(5, :),'r'); hold on;
title('Extended Kalman Filter', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('Ground Truth','EKF');
grid on;
axis equal;

% Auto save graph
saveas(gcf, 'Extended_Kalman_Filter.png');