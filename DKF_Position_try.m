clear all;
close all;
clc;

% Define the number of points and grid dimensions
numPoints = 6;      % Set the number of points you want to generate
dimgrid = [1000 1000];   % Define the height of the grid
kp = 20;

points = zeros(numPoints,2);

% Generate random positions for each point
x = rand(numPoints, 1) * dimgrid(1);  % Random x coordinates
y = rand(numPoints, 1) * dimgrid(2);  % Random y coordinates

points = [x,y];

% Plot the points on a figure
figure(1);
scatter(points(:,1),points(:,2));
axis([0 dimgrid(1) 0 dimgrid(2)]); 
xlabel('X Coordinate');
ylabel('Y Coordinate');
title(sprintf('Randomly Placed %d Points', numPoints));

% Compute and plot Voronoi tessellation
[vx, vy] = voronoi(points(:,1), points(:,2));

figure(2);
hold on;
plot(vx, vy, 'r-', 'LineWidth', 1.5);
axis([0 dimgrid(1) 0 dimgrid(2)]); 
scatter(points(:,1),points(:,2));
title('Voronoi Tassellation');

%% Compute Voronoi tessellation using voronoin
[areas,centroids,vel] = voronoi_function(dimgrid,points,kp);

% Display the areas and centroids
sum_areas = sum(areas);
for i = 1:length(areas)
    fprintf('areas%d: %f\n',i,areas(i));
end
disp(sum_areas)
for i = 1:length(areas)
    fprintf('centroids coordinates %d: [%f,%f]\n',i,centroids(i,1),centroids(i,2));
end
disp(sum_areas)

%% Uncertainty
% GPS
ProbGPS = 0.8 + (0.95 - 0.8) * rand(numPoints, 1);
sigma_gps = 2 + (5 - 2) * rand(numPoints, 1);

% Control input
sigma_u = 0.5 + (2 - 0.5) * rand(numPoints, 1);

% Radar Noise
sigma_radar = 0.5 + (2 - 0.5) * rand(numPoints, 1);

%% Initialization Kalman Filter
nx_Est = zeros(numPoints, 2);
P = 100 * ones(2, 2, numPoints);
for i = 1:numPoints
    nx_Est(i,:) = [points(i,1), points(i,2)];
    P(:,:,i) = 100 * eye(2);
end

%% Consensus Param
numIterations = 10; % Numero di iterazioni per il consenso
alpha = 0.5; % Fattore di aggiornamento per il consenso

%% Simulation 

dt = 0.01;
T_sim = 50;

trajectories = zeros(numPoints,2,T_sim);
trajectories_Est = zeros(numPoints,2,T_sim);  % Preallocate trajectories_Est array
trajectories_GPS = zeros(numPoints,2,T_sim);  % Preallocate trajectories_GPS array
trajectories_consensus = zeros(numPoints,2,T_sim);  % Preallocate trajectories_consensus array  

nx = points;
trajectories(:,:,1) = nx;
trajectories_Est(:,:,1) = nx_Est;
trajectories_GPS(:,:,1) = nx;
trajectories_consensus(:,:,1) = nx_Est;

% Creazione di una matrice di connessione (ogni drone comunica con i vicini diretti)
adjacency_matrix = ones(numPoints, numPoints) - eye(numPoints); % Tutti connessi (puoi personalizzare)

% Prepare figure for simulation
figure(3);
colors = lines(numPoints);
hold on;
axis([0 dimgrid(1) 0 dimgrid(2)]);
xlabel('X Coordinate');
ylabel('Y Coordinate');
title('Lloyd Simulation');

for t = 2:T_sim
    % Compute Voronoi tessellation and centroids
    [areas, centroids, vel] = voronoi_function(dimgrid, nx, kp);

    % Update positions using 2D velocity vectors
    nx = nx + vel * dt;
    % Save the updated positions into the trajectories array
    trajectories(:, :, t) = nx;

    % Kalman Filter
    for i = 1:numPoints
        GPS = nx(i, :)' + randn(2, 1) * sigma_gps(i);
        u_bar = vel(i, :)' + randn(2, 1) * sigma_u(i);
        [nx_Est(i,:), P(:,:,i)] = kalman_planes_function(nx_Est(i,:)', P(:,:,i), u_bar, sigma_u(i), sigma_gps(i), GPS, ProbGPS(i), dt);

        trajectories_GPS(i, :, t) = GPS';
        trajectories_U(i, :, t) = (nx(i, :)' + u_bar * dt)';
    end
    trajectories_Est(:,:,t) = nx_Est;

    %% Consensus Algorithm
    % Compute Distance Matrix with noise
    D = pdist2(nx_Est,nx_Est) + randn(numPoints,numPoints) * sigma_radar;
    % Compute Consensus Algorithm
    [nx_Est, P] = consensus_algorithm_function_try(nx_Est, P, adjacency_matrix, alpha);

    % Save consensus trajectories
    trajectories_consensus(:,:,t) = nx_Est;

    % Clear the figure and replot everything using arrays of points
    clf; hold on;
    for i = 1:numPoints
        % Extract the trajectory so far (squeeze the slice into a 2D array)
        traj = squeeze(trajectories(i, :, 1:t));
        plot(traj(1, :), traj(2, :), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
        % Plot the current drone position as a marker
        plot(nx(i, 1), nx(i, 2), 'o', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
    end
    
    drawnow;  % Force MATLAB to update the figure
    % Optionally add a pause (e.g., pause(0.01)) to slow down the simulation for visualization
end

% Plot the estimated trajectories vs Real
figure(4);
hold on;
for i = 1:numPoints
    % Extract the real and estimated trajectories
    real_traj = squeeze(trajectories(i, :, :));
    est_traj = squeeze(trajectories_Est(i, :, :));
    
    % Plot the estimated trajectory
    plot(est_traj(1, :), est_traj(2, :), '--', 'Color', colors(i,:), 'LineWidth', 2);
    
    % Plot the real trajectory
    plot(real_traj(1, :), real_traj(2, :), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
    
    % Plot the consensus trajectory
    plot(squeeze(trajectories_consensus(i,1,:)), squeeze(trajectories_consensus(i,2,:)), '-.', 'Color', colors(i,:), 'LineWidth', 1);
end
xlabel('X Coordinate');
ylabel('Y Coordinate');
title('Estimated vs Real Trajectories');
legend('Real Trajectory', 'Estimated Trajectory','Consensus Trajectory');

% Real And Stimated Trajectories
% Plot the X coordinates in one figure
figure(5);
hold on;
for i = 1:numPoints
    subplot( 1 ,numPoints, i);
    plot(squeeze(trajectories_Est(i,1,:)), 'r', 'LineWidth', 1.5);
    hold on;
    plot(squeeze(trajectories(i,1,:)), 'b', 'LineWidth', 1.5);
    plot(squeeze(trajectories_GPS(i,1,:)), 'g', 'LineWidth', 1.5);
    plot(squeeze(trajectories_consensus(i,1,:)), '--m', 'LineWidth', 1.5);
    xlabel('Time');
    ylabel(sprintf('X Coordinate (Point %d)', i));
    title(sprintf('Trajectories Point %d', i));
    legend('Estimated', 'Real', 'GPS', 'Consensus');
end

% Plot the Y coordinates in another figure
figure(6);
hold on;
for i = 1:numPoints
    subplot(1 ,numPoints, i);
    plot(squeeze(trajectories_Est(i,2,:)), 'r', 'LineWidth', 1.5);
    hold on;
    plot(squeeze(trajectories(i,2,:)), 'b', 'LineWidth', 1.5);
    plot(squeeze(trajectories_GPS(i,2,:)), 'g', 'LineWidth', 1.5);
    plot(squeeze(trajectories_consensus(i,2,:)), '--m', 'LineWidth', 1.5);
    xlabel('Time');
    ylabel(sprintf('Y Coordinate (Point %d)', i));
    title(sprintf('Trajectories Point %d', i));
    legend('Estimated', 'Real', 'GPS', 'Consensus');    
end