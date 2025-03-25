clear all;
close all;
clc;

% Parameters
DO_SIMULATION = true;
INITIAL_VALUES = false;
PLOT_TRAJECTORIES = false;
SAVE_TRAJECTORIES = false;
vel_max = 800; 
inc_threshold1 = 30;  % Distance that has to be reach from the fire 1 
inc_threshold2 = 15;  % Distance that has to be reach from the fire 2
wat_threshold = 20;   % Distance that has to be reach from the water source to refill
refill_time = 20;

numDrones = 7;      % Set the number of drones
dimgrid = [500 500];   % Define the dimensions of the grid
kp = 20;   % Proportional gain for the Lyoid control 

points = zeros(numDrones,2);

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
sigma_fire1 = 30;
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

%% Simulation 

dt = 0.01;
T_sim = 500;

trajectories = zeros(numDrones,2,T_sim);

nx = points;
trajectories(:,:,1) = nx;

% Prepare figure for simulation
figure(5);
colors = lines(numDrones);
hold on;
axis([0 dimgrid(1) 0 dimgrid(2)]);
xlabel('X Coordinate');
ylabel('Y Coordinate');
title('Lloyd Simulation');

if DO_SIMULATION
    wait_counter = -ones(numDrones,1);
    count = -ones(numDrones,1);

    for i = 1:numDrones
        if count(i) == 0
            count(i) = -1;
            wait_counter(i) = -1;
        end
    end

    for t = 2:T_sim

        dist_inc1 = pdist2(pos_fire1, nx);
        dist_inc2 = pdist2(pos_fire2, nx);
        dist_wat = pdist2(pos_water, nx);

        for i= 1:numDrones
            if(dist_inc1(i) <= inc_threshold1 || dist_inc2(i) <= inc_threshold2)
                status(i) = 2;
            end
            if dist_wat(i) <= wat_threshold 
                status(i) = 1;
                if wait_counter(i) == -1 && count(i) == -1
                    wait_counter(i) = refill_time; % time needed for the refill
                    count(i) = 5;
                end
            end
        end

        % Compute Voronoi tessellation and centroids
        [areas, centroids, vel] = voronoi_function(dimgrid, nx, kp, G_fire, G_water, status);

        % Impose a maximum velocity
        vel = sign(vel) .* min(abs(vel), vel_max);

        for i = 1:numDrones
            if wait_counter(i) > 0
                vel(i,:) = [0,0];
                wait_counter(i) = wait_counter(i) - 1;
            elseif count(i) == 0
                count(i) = -1;
                wait_counter(i) = -1;
            elseif wait_counter(i) == 0
                count(i) = count(i) - 1;
            end
        end

        % Update positions using 2D velocity vectors
        nx = nx + vel * dt;

        cla;
        hold on;
        
        if SAVE_TRAJECTORIES
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
            plot(nx(i, 1), nx(i, 2), 'o', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
            
        end
        voronoi(nx(:,1),nx(:,2));
        
        plot(x_fire1,y_fire1,'x','Color', 'r', 'MarkerSize', sigma_fire1)
        plot(x_fire2,y_fire2,'x','Color', 'r', 'MarkerSize', sigma_fire2)
        plot(x_water,y_water,'o','Color', 'b', 'MarkerSize', sigma_water)

        drawnow;  % Force MATLAB to update the figure
        % Optionally add a pause (e.g., pause(0.01)) to slow down the simulation for visualization
    end
end