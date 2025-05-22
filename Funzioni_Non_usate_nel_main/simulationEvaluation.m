% simulationEvaluation.m
% This script evaluates the performance of a UAV-based fire suppression simulation.
% It runs two sets of experiments: 
%   1) Varying the number of UAVs and recording key performance metrics.
%   2) Varying the spread parameter (sigma) of Fire 1 while keeping the number of UAVs fixed.
% Results are saved to CSV files and visualized using plots.

clear;
clc;
close all;

% Simulation evaluation script

numUAV_range = 3:20;
sigma_fire1_range = 10:10:100;

% First loop: only number of UAVs (sigma_fire1 is default)
results = zeros(length(numUAV_range), 4); % mean_drops_inTime, totalDrops, drops_f1, drops_f2

for idx = 1:length(numUAV_range)
    fprintf('Running simulation with %d UAVs...\n', numUAV_range(idx));
    numUAV = numUAV_range(idx);
    [mean_drops_inTime, totalDrops, drops_f1, drops_f2] = All_project_function(numUAV);
    results(idx, :) = [mean_drops_inTime, totalDrops, drops_f1, drops_f2];
    fprintf('\nSimulation with %d UAVs completed.\n', numUAV);
    fprintf('-------------------\n');
end

% Save results of the first loop
filename = 'simulation_results.csv';
writematrix([numUAV_range', results], filename);

% Plot results of the first loop
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

% Second loop: varying sigma_fire1 and number of UAVs set to 5
results_sigma = zeros(length(sigma_fire1_range), 6); % UAV, sigma_fire1, mean_drops_inTime, totalDrops, drops_f1, drops_f2
numUAV = 5;
sigma_fire2 = 50; % Fixed sigma_fire2

for j = 1:length(sigma_fire1_range)
    sigma_fire1 = sigma_fire1_range(j);
    fprintf('Running simulation with %d UAVs, sigma_fire1=%d, sigma_fire2=%d...\n', numUAV, sigma_fire1, sigma_fire2);
    [mean_drops_inTime, totalDrops, drops_f1, drops_f2] = All_project_function(numUAV, sigma_fire1, sigma_fire2);
    results_sigma(j, :) = [numUAV, sigma_fire1, mean_drops_inTime, totalDrops, drops_f1, drops_f2];
    fprintf('\nSimulation with %d UAVs, sigma_fire1=%d, sigma_fire2=%d completed.\n', numUAV, sigma_fire1, sigma_fire2);
    fprintf('-------------------\n');
end

% Save results of the second loop
filename_sigma = 'simulation_results_sigma.csv';
writematrix(results_sigma, filename_sigma);

% Plot results of the second loop
figure;
subplot(2,2,1);
plot(sigma_fire1_range, results_sigma(:,3), '-o');
xlabel('Sigma Fire 1');
ylabel('Mean time between drops');
title('Mean time between drops vs Sigma Fire 1');

subplot(2,2,2);
plot(sigma_fire1_range, results_sigma(:,4), '-o');
xlabel('Sigma Fire 1');
ylabel('Total number of drops');
title('Total number of drops vs Sigma Fire 1');

subplot(2,2,3);
plot(sigma_fire1_range, results_sigma(:,5), '-o');
xlabel('Sigma Fire 1');
ylabel('Drops on Fire 1');
title('Drops on Fire 1 vs Sigma Fire 1');

subplot(2,2,4);
plot(sigma_fire1_range, results_sigma(:,6), '-o');
xlabel('Sigma Fire 1');
ylabel('Drops on Fire 2');
title('Drops on Fire 2 vs Sigma Fire 1');

sgtitle('Simulation results as a function of Sigma Fire 1 (UAV=5, Sigma Fire 2=50)');
