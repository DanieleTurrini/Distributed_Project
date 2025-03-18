clear all;
close all;
clc;

% Parameters
DO_SIMULATION = true;
INITIAL_VALUES = false;
PLOT_TRAJECTORIES = false;

SAVE_TRAJECTORIES = true;
PLOT_TRAJECTORIES_COMPARISON = true;   

vel_max = 800;
inc_threshold1 = 20;
inc_threshold2 = 20;
wat_threshold = 10;

% Define the number of points and grid dimensions
numPoints = 3;      % Set the number of points you want to generate
dimgrid = [500 500];   % Define the height of the grid
kp = 20;

points = zeros(numPoints,2);

% Generate random positions for each point
x = rand(numPoints, 1) * 100;   % Random x coordinates
y = rand(numPoints, 1) * 100;  % Random y coordinates

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

%% Funzione densità per incendi 

% Definizione dei parametri della densità
x_fire1 = 400;
y_fire1 = 400;
x_fire2 = 450;
y_fire2 = 50;

x_water = 50;
y_water = 50;

pos_fire1 = [x_fire1, y_fire1];
pos_fire2 = [x_fire2, y_fire2];
pos_water = [x_water, y_water];

sigma_fire1 = 40;
sigma_fire2 = 15;
sigma_water = 20;

% Creazione della griglia di punti
[x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));

% Calcolo della distribuzione gaussiana
G_fire = exp(-(((x_m - x_fire1).^2) / (2 * sigma_fire1^2) + ((y_m - y_fire1).^2) / (2 * sigma_fire1^2))) + exp(-(((x_m - x_fire2).^2) / (2 * sigma_fire2^2) + ((y_m - y_fire2).^2) / (2 * sigma_fire2^2)));
G_water = exp(-(((x_m - x_water).^2) / (2 * sigma_water^2) + ((y_m - y_water).^2) / (2 * sigma_water^2)));

% Visualizzazione della matrice
imagesc(G_fire);
colormap jet;
colorbar;
title('Funzione densità: Incendi');

% Visualizzazione in 3D
figure(3);
surf(x_m, y_m, G_fire);
shading interp; % Per rendere la superficie più liscia
colormap jet;
colorbar;
xlabel('X');
ylabel('Y');
zlabel('Densità');
title('Funzione densità: Incendi');
drawnow;
hold off;

status = ones(numPoints,1); 
% status = 1 il drone è carico di acqua e sta andando verso l'incendio
% status = 2 il drone è scarico di acqua e sta andando a rifornirsi 

%% Initial Voronoi Tassellation
[areas,centroids,vel] = voronoi_function(dimgrid,points,kp,G_fire,G_water,status);
if INITIAL_VALUES
    sum_areas = sum(areas);
    for i = 1:length(areas)
        fprintf('Initial areas %d: %f\n',i,areas(i));
    end
    disp(sum_areas)
    for i = 1:length(areas)
        fprintf('Initial centroids coordinates %d: [%f,%f]\n',i,centroids(i,1),centroids(i,2));
    end
    disp(sum_areas)
end

%% Uncertanties for Kalman Filter
% Gps
ProbGPS = 0.8 + (0.95 - 0.8) * rand(numPoints, 1);
sigma_gps = 20;

% Control input
sigma_u = 10;

%% Initialization Kalman Filter
nx_Est = zeros(numPoints, 2);
P = 100 * ones(2, 2, numPoints);
for i = 1:numPoints
    nx_Est(i,:) = [points(i,1), points(i,2)];
    P(:,:,i) = 100 * eye(2);
end
GPS = zeros(numPoints, 2);

%% Simulation 

dt = 0.01;
T_sim = 100;

trajectories = zeros(numPoints,2,T_sim);
trajectories_Est = zeros(numPoints,2,T_sim);  % Preallocate trajectories_Est array
trajectories_GPS = zeros(numPoints,2,T_sim);  % Preallocate trajectories_GPS array

nx = points;
trajectories(:,:,1) = nx;
trajectories_Est(:,:,1) = nx_Est;
trajectories_GPS(:,:,1) = nx_Est;

% Colors for plotting
colors = lines(numPoints);

figure(4);
clf;
axis([0 dimgrid(1) 0 dimgrid(2)]);
xlabel('X Coordinate');
ylabel('Y Coordinate');
title('Lloyd Simulation Real');
hold on;


if DO_SIMULATION
    for t = 2:T_sim

        dist_inc1 = pdist2(pos_fire1, nx);
        dist_inc2 = pdist2(pos_fire2, nx);
        dist_wat = pdist2(pos_water, nx);

        for i= 1:numPoints
            if(dist_inc1(i) <= inc_threshold1 || dist_inc2(i) <= inc_threshold2)
                status(i) = 2;
            end
            if(dist_wat(i) <= wat_threshold)
                status(i) = 1;
            end
        end

        % Compute Voronoi tessellation and centroids
        [areas, centroids, vel] = voronoi_function(dimgrid, nx, kp, G_fire, G_water, status);

        % Impose a maximum velocity
        vel = sign(vel) .* min(abs(vel), vel_max);
        
        % Update positions using 2D velocity vectors
        nx = nx + vel * dt;

        %% Consensus Algorithm

%{
         % Each point updates its estimate of the positions of all other points
        for i = 1:numPoints
            for j = 1:numPoints
            if i ~= j
                % Update the estimate of point j's position using a simple averaging consensus algorithm
                nx_Est(i,:) = nx_Est(i,:) + 0.1 * (nx_Est(j,:) - nx_Est(i,:));
            end
            end
        end 
%}

        %% Kalman Filter

        % Voronoi with Estimated positions
        [areas_est, centroids_est, vel_est] = voronoi_function(dimgrid, nx_Est, kp, G_fire, G_water, status);
        % Impose a maximum velocity
        vel_est = sign(vel_est) .* min(abs(vel_est), vel_max);
        
        % Compute Kalman for Every Drone
        for i = 1:numPoints
            GPS(i,:) = nx(i, :)' + randn(2, 1) * sigma_gps;
            u_bar = vel_est(i, :)' + randn(2, 1) * sigma_u;

            [nx_Est(i,:), P(:,:,i)] = kalman_planes_function(nx_Est(i,:)', P(:,:,i), u_bar, sigma_u, sigma_gps, GPS(i,:)', ProbGPS(i), dt);
        end
        %% Save the updated positions into the trajectories array
        if SAVE_TRAJECTORIES
            % Save the updated positions into the trajectories array
            trajectories(:, :, t) = nx;
            trajectories_Est(:,:,t) = nx_Est;
            trajectories_GPS(:,:,t) = GPS;
        end

        %% PLOTS
        % Real Voronoi
        % figure(4);
        % clf;
        % axis([0 dimgrid(1) 0 dimgrid(2)]);
        % xlabel('X Coordinate');
        % ylabel('Y Coordinate');
        % title('Lloyd Simulation Real');
        % hold on;
        

        cla;
        % Plot the trajectory for each drone up to the current time
        for i = 1:numPoints
            if PLOT_TRAJECTORIES
                % Extract the trajectory so far (squeeze the slice into a 2D array)
                traj = squeeze(trajectories(i, :, 1:t));
                plot(traj(1, :), traj(2, :), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
            end
            % Plot the current drone position as a marker
            plot(nx(i, 1), nx(i, 2), 'o', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
            voronoi(nx(:,1),nx(:,2));
        end
        
        plot(x_fire1,y_fire1,'x','Color', 'r', 'MarkerSize', sigma_fire1)
        plot(x_fire2,y_fire2,'x','Color', 'r', 'MarkerSize', sigma_fire2)
        plot(x_water,y_water,'o','Color', 'b', 'MarkerSize', sigma_water)

        % drawnow;  % Force MATLAB to update the figure

   
        % % Estimated Voronoi
        % figure(5);
        % clf;
        % axis([0 dimgrid(1) 0 dimgrid(2)]);
        % xlabel('X Coordinate');
        % ylabel('Y Coordinate');
        % title('Lloyd Simulation Estimated');
        % hold on;

        % Plot the trajectory for each drone up to the current time
        for i = 1:numPoints
            if PLOT_TRAJECTORIES
                % Extract the trajectory so far (squeeze the slice into a 2D array)
                traj = squeeze(trajectories_Est(i, :, 1:t));
                plot(traj(1, :), traj(2, :), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
            end
            % Plot the current drone position as a marker
            plot(nx_Est(i, 1), nx_Est(i, 2), 'o', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
            voronoi(nx_Est(:,1),nx_Est(:,2));
        end

        plot(x_fire1,y_fire1,'x','Color', 'r', 'MarkerSize', sigma_fire1)
        plot(x_fire2,y_fire2,'x','Color', 'r', 'MarkerSize', sigma_fire2)
        plot(x_water,y_water,'o','Color', 'b', 'MarkerSize', sigma_water)

        drawnow;

        % Optionally add a pause (e.g., pause(0.01)) to slow down the simulation for visualization
    end

end

%% Plot the estimated trajectories vs Real
if PLOT_TRAJECTORIES_COMPARISON

    for i = 1:numPoints
        % Plot the estimated trajectories vs Real
        figure(6);
        hold on;
        % Extract the real and estimated trajectories
        real_traj = squeeze(trajectories(i, :, :));
        est_traj = squeeze(trajectories_Est(i, :, :));

        % Plot the estimated trajectory
        plot(est_traj(:, 1), est_traj(:, 2), '--', 'Color', colors(i,:), 'LineWidth', 2);

        % Plot the real trajectory
        plot(real_traj(:, 1), real_traj(:, 2), '-', 'Color', colors(i,:), 'LineWidth', 1.5);

        title(sprintf('Estimated vs Real Trajectories point %d', i));
        xlabel('X Coordinate');
        ylabel('Y Coordinate');
        legend('Estimated', 'Real');
        pause(1);
        hold off;
    end

    % Posizioni X reale vs stimata
    points_to_plot = 3;
    figure(7);
    for i = 1:points_to_plot
        subplot(1, points_to_plot, i);
        hold on;
        plot(squeeze(trajectories_GPS(i,1,:)), '--g', 'LineWidth', 1);
        plot(squeeze(trajectories_Est(i,1,:)), 'r', 'LineWidth', 1.5);
        plot(squeeze(trajectories(i,1,:)), 'b', 'LineWidth', 1.5);
        xlabel('Time');
        ylabel(sprintf('X Coordinate (Point %d)', i));
        title(sprintf('Trajectories Point %d', i));
        legend('GPS', 'Estimated', 'Real');
        hold off;
    end

    % Posizioni Y reale vs stimata
    figure(8);
    for i = 1:points_to_plot
        subplot(1, points_to_plot, i);
        hold on;
        plot(squeeze(trajectories_GPS(i,2,:)), '--g', 'LineWidth', 1);
        plot(squeeze(trajectories_Est(i,2,:)), 'r', 'LineWidth', 1.5);
        plot(squeeze(trajectories(i,2,:)), 'b', 'LineWidth', 1.5);
        xlabel('Time');
        ylabel(sprintf('Y Coordinate (Point %d)', i));
        title(sprintf('Trajectories Point %d', i));
        legend('GPS', 'Estimated', 'Real');
        hold off;
    end



end