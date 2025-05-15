clear; close all; clc;


numUAV = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14];
times  = [235, 162, 143, 129, 139, 135, 138, 132, 122, 110, 96, 95];
drops_f1 = [6,9,11,15,12,11,11,11,14,16,19,20];
drops_f2 = [3,3,4,3,5,6,6,6,5,5,5,4];


figure(1); hold on;
plot(numUAV, times, 'bo', 'MarkerFaceColor','b');

% draw a vertical “stem” from y=0 up to each time
for i = 1:numel(numUAV)
    line( [numUAV(i) numUAV(i)], [0 times(i)], ...
          'Color','b', 'LineStyle','-', 'LineWidth', 0.5 );
end

xlabel('Number of UAVs');
ylabel('Time distance between consecutive drops');
title('System Efficiency as Function of UAV number');
grid on;
hold off;



% Prepare data matrix (rows correspond to each numUAV, columns to the two series)
Y = [drops_f1; drops_f2].';

% Plot
figure(2);

h = bar(numUAV, Y, 'grouped');
ylim([0 22])
% Set custom colors
h(1).FaceColor = [0.2 0.6 0.8];   % e.g. blueish for drops_f1
h(2).FaceColor = [0.9 0.4 0.4];   % e.g. reddish  for drops_f2

% Labels and legend
xlabel('Number of UAVs');
ylabel('Number of Drops');
title('Drops per Fire for Varying UAV Fleet Sizes');
legend({'Fire 1','Fire 2'}, 'Location','northwest');

% Optional: grid, tight axes
grid on;
set(gca,'XTick', numUAV);
xlim([min(numUAV)-1, max(numUAV)+1]);