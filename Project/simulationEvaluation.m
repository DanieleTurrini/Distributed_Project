clear;
clc;
close all;

% Simulation evaluation script

numUAV_range = 3:20;
results = zeros(length(numUAV_range), 4); % mean_drops_inTime, totalDrops, drops_f1, drops_f2

for idx = 1:length(numUAV_range)
    fprintf('Running simulation with %d UAVs...\n', numUAV_range(idx));
    numUAV = numUAV_range(idx);
    [mean_drops_inTime, totalDrops, drops_f1, drops_f2] = All_project_function(numUAV);
    results(idx, :) = [mean_drops_inTime, totalDrops, drops_f1, drops_f2];
    fprintf('\n Simulation with %d UAVs completed.\n', numUAV);
    fprintf('-------------------\n');
end

% Save results to a CSV file (overwrite if it already exists)
filename = 'simulation_results.csv';
writematrix([numUAV_range', results], filename);

% Plot results
figure;
subplot(2,2,1);
plot(numUAV_range, results(:,1), '-o');
xlabel('Number of UAVs');
ylabel('Mean time between drops');
title('Mean time between drops');

subplot(2,2,2);
plot(numUAV_range, results(:,2), '-o');
xlabel('Number of UAVs');
ylabel('Total number of drops');
title('Total number of drops');

subplot(2,2,3);
plot(numUAV_range, results(:,3), '-o');
xlabel('Number of UAVs');
ylabel('Drops on Fire 1');
title('Drops on Fire 1');

subplot(2,2,4);
plot(numUAV_range, results(:,4), '-o');
xlabel('Number of UAVs');
ylabel('Drops on Fire 2');
title('Drops on Fire 2');

sgtitle('Simulation results as a function of the number of UAVs');
