clear all;
close all;
clc;

% Parameters
DO_SIMULATION = true;
INITIAL_VALUES = false;
PLOT_TRAJECTORIES = false;

acc_max = 100; 
inc_threshold1 = 15;  % Distance that has to be reach from the fire 1 
inc_threshold2 = 15;  % Distance that has to be reach from the fire 2
wat_threshold = 20;   % Distance that has to be reach from the water source to refill
refill_time = 20;

dt = 0.01;
T_sim = 500;

numDrones = 7;      % Set the number of drones
dimgrid = [500 500];   % Define the dimensions of the grid
Kp = 20;   % Proportional gain for the Lyoid control 
Ki = 10;               % Guadagno integrale
Kd = 10;              % Guadagno derivativo

% Matrice di transizione di stato
A = [1, 0, dt, 0;
     0, 1, 0, dt;
     0, 0, 1, 0;
     0, 0, 0, 1];

% Matrice di controllo 
B = [0.5 * dt^2,          0;
              0, 0.5 * dt^2;
             dt,          0;
              0,         dt];

stati = zeros(4,1,numDrones);

% Generate random starting positions for each point
x = rand(numDrones, 1) * 100;   % Random x coordinates
y = 100 + rand(numDrones, 1) * 100;  % Random y coordinates

points = [x,y];

% Plot the points on a figure
figure(1);
scatter(points(:,1),points(:,2));
axis([0 dimgrid(1) 0 dimgrid(2)]); 
xlabel('X Coordinate');
ylabel('Y Coordinate');
title(sprintf('Initial position of the %d drones', numDrones));

% Compute and plot Voronoi tessellation
[vx, vy] = voronoi(points(:,1), points(:,2));

figure(2);
hold on;
plot(vx, vy, 'r-', 'LineWidth', 1.5);
axis([0 dimgrid(1) 0 dimgrid(2)]); 
scatter(points(:,1),points(:,2));
title('Voronoi Tassellation');

%% Density function for fires

% Fires Positions
x_fire1 = 400;
y_fire1 = 400;
x_fire2 = 450;
y_fire2 = 50;

% Water Positions
x_water = 50;
y_water = 50;

pos_fire1 = [x_fire1, y_fire1];
pos_fire2 = [x_fire2, y_fire2];
pos_water = [x_water, y_water];

% Standard deviations of fires and water source
sigma_fire1 = 15;
sigma_fire2 = 15;
sigma_water = 20;

% Creazione della griglia di punti
[x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));

% Calcolo della distribuzione gaussiana
G_fire = exp(-(((x_m - x_fire1).^2) / (2 * sigma_fire1^2) + ((y_m - y_fire1).^2) / (2 * sigma_fire1^2))) + exp(-(((x_m - x_fire2).^2) / (2 * sigma_fire2^2) + ((y_m - y_fire2).^2) / (2 * sigma_fire2^2)));
G_water = exp(-(((x_m - x_water).^2) / (2 * sigma_water^2) + ((y_m - y_water).^2) / (2 * sigma_water^2)));

% Visualizzazione in 3D
figure(3);
surf(x_m, y_m, G_fire);
shading interp; % Per rendere la superficie più liscia
colormap jet;
colorbar;
xlabel('X');
ylabel('Y');
zlabel('Densità');
title('Funzione densità Incendi');
drawnow;
hold off;

figure(4);
surf(x_m, y_m, G_water);
shading interp; % Per rendere la superficie più liscia
colormap jet;
colorbar;
xlabel('X');
ylabel('Y');
zlabel('Densità');
title('Funzione densità Sorgente Acqua');
drawnow;
hold off;

status = ones(numDrones,1); 
% status = 1 il drone è carico di acqua e sta andando verso l'incendio
% status = 2 il drone è scarico di acqua e sta andando a rifornirsi 

nx = points;

trajectories = zeros(numDrones,2,T_sim/dt);
trajectories(:,:,1) = nx;

for i = 1:numDrones
    stati(1,1,i) = points(i,1);
    stati(2,1,i) = points(i,2);
end

%% Initial Voronoi Tassellation
[areas,centroids,acc_des] = voronoi_function(dimgrid,stati, Kp, Ki, Kd, dt, G_fire,G_water,status);
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

%% Simulation 

% Prepare figure for simulation
figure(5);
colors = lines(numDrones);
hold on;
axis([0 dimgrid(1) 0 dimgrid(2)]);
xlabel('X Coordinate');
ylabel('Y Coordinate');
title('Lloyd Simulation');

if DO_SIMULATION

    for t = 2:dt:T_sim

        dist_inc1 = pdist2(pos_fire1, nx);
        dist_inc2 = pdist2(pos_fire2, nx);
        dist_wat = pdist2(pos_water, nx);

        for i= 1:numDrones
            if(dist_inc1(i) <= inc_threshold1 || dist_inc2(i) <= inc_threshold2)
                status(i) = 2;
            end
            if dist_wat(i) <= wat_threshold 
                status(i) = 1;
            end
        end

        % Compute Voronoi tessellation and centroids
        [areas, centroids, acc_des] = voronoi_function(dimgrid, stati,  Kp, Ki, Kd, dt, G_fire, G_water, status);

        % Impose a maximum acceleration
        acc = sign(acc_des) .* min(abs(acc_des), acc_max);

        for k = 1:numDrones
            stati(:,1,k) = A * stati(:,1,k) + B * acc(k,:)';

            if PLOT_TRAJECTORIES
                trajectories(k,:,t) = [stati(1,1,k), stati(2,1,k)];
            end
        end

        cla;
        hold on;
        
        if PLOT_TRAJECTORIES
            % Save the updated positions into the trajectories array
            trajectories(:, :, t) = nx;
        end
        
        % Plot the trajectory for each drone up to the current time
        for i = 1:numDrones
            if PLOT_TRAJECTORIES
                % Extract the trajectory so far (squeeze the slice into a 2D array)
                traj = squeeze(trajectories(i, :, 1:t));
                plot(traj(1, :), traj(2, :), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
            end
            % Plot the current drone position as a marker
            plot(squeeze(stati(1,1,i)), squeeze(stati(2,1,i)), 'o', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
            
        end
        voronoi(squeeze(stati(1,1,:)), squeeze(stati(2,1,:)));
        
        plot(x_fire1,y_fire1,'x','Color', 'r', 'MarkerSize', sigma_fire1)
        plot(x_fire2,y_fire2,'x','Color', 'r', 'MarkerSize', sigma_fire2)
        plot(x_water,y_water,'o','Color', 'b', 'MarkerSize', sigma_water)

        drawnow;  % Force MATLAB to update the figure
        % Optionally add a pause (e.g., pause(0.01)) to slow down the simulation for visualization
    end
end