clear all;
close all;
clc;

% Parameters
DO_SIMULATION = true;
INITIAL_VALUES = false;
PLOT_TRAJECTORIES = false;
SAVE_TRAJECTORIES = false;
vel_lin_max = 300; 
vel_ang_max = 20; 
inc_threshold1 = 15;  % Distance that has to be reach from the fire 1 
inc_threshold2 = 15;  % Distance that has to be reach from the fire 2
wat_threshold = 60;   % Distance that has to be reach from the water source to refill
refill_time = 20;

dt = 0.01;
T_sim = 500;

numDrones = 10;      % Set the number of drones
dimgrid = [500 500];   % Define the dimensions of the grid
Kp = 50;   % Proportional gain for the Lyoid control 
Ka = 15;
Ke = 10;

points = zeros(numDrones,2);

% Generate random starting positions for each point
x = rand(numDrones, 1) * 100;   % Random x coordinates
y = 100 + rand(numDrones, 1) * 100;  % Random y coordinates
theta= rand(numDrones,1)*2*pi;

points = [x,y];
stati = [x,y,theta];

A = [1, 0, 0;
     0, 1, 0;
     0, 0, 1];

B = @(theta) [ dt * cos(theta), 0;
               dt * sin(theta), 0;
               0,               dt];

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
sigma_water = 60;

% Creazione della griglia di punti
[x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));

% Calcolo della distribuzione gaussiana
G_fire = exp(-(((x_m - x_fire1).^2) / (2 * sigma_fire1^2) + ((y_m - y_fire1).^2) / (2 * sigma_fire1^2))) + 0.1*exp(-(((x_m - x_fire2).^2) / (2 * sigma_fire2^2) + ((y_m - y_fire2).^2) / (2 * sigma_fire2^2)));
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
[areas,centroids,vel] = voronoi_function_plane(dimgrid,stati,Kp,Ka,Ke,G_fire,G_water,status);
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

trajectories = zeros(numDrones,2,T_sim);

trajectories(:,:,1) = stati(:,1:2);

% Prepare figure for simulation
figure(5);
colors = lines(numDrones);
hold on;
axis([0 dimgrid(1) 0 dimgrid(2)]);
xlabel('X Coordinate');
ylabel('Y Coordinate');
title('Lloyd Simulation');

if DO_SIMULATION
    % wait_counter = -ones(numDrones,1);
    % count = -ones(numDrones,1);
    % 
    % for i = 1:numDrones
    %     if count(i) == 0
    %         count(i) = -1;
    %         wait_counter(i) = -1;
    %     end
    % end

    for t = 2:dt:T_sim

        dist_inc1 = pdist2(pos_fire1, stati(:,1:2));
        dist_inc2 = pdist2(pos_fire2, stati(:,1:2));
        dist_wat = pdist2(pos_water, stati(:,1:2));

        for i= 1:numDrones
            if(dist_inc1(i) <= inc_threshold1 || dist_inc2(i) <= inc_threshold2)
                status(i) = 2;
            end
            if dist_wat(i) <= wat_threshold 
                status(i) = 1;
                % if wait_counter(i) == -1 && count(i) == -1
                %     wait_counter(i) = refill_time; % time needed for the refill
                %     count(i) = 5;
                % end
            end
        end

        % Compute Voronoi tessellation and centroids
        [areas, centroids, vel] = voronoi_function_plane(dimgrid, stati, Kp, Ka, Ke, G_fire, G_water, status);

        % Impose a maximum velocity
        vel(:,1) = sign(vel(:,1)) .* min(abs(vel(:,1)), vel_lin_max);
        vel(:,2) = sign(vel(:,2)) .* min(abs(vel(:,2)), vel_ang_max);

        % for i = 1:numDrones
        %     if wait_counter(i) > 0
        %         vel(i,:) = [0,0];
        %         wait_counter(i) = wait_counter(i) - 1;
        %     elseif count(i) == 0
        %         count(i) = -1;
        %         wait_counter(i) = -1;
        %     elseif wait_counter(i) == 0
        %         count(i) = count(i) - 1;
        %     end
        % end

        % Update positions using 2D velocity vectors
        
        for k = 1:numDrones
            stati(k,:) = stati(k,:) * A' + vel(k,:) * B(stati(k,3))';
            % stati(k,1) = stati(k,1) + vel(k,1) * dt * cos(stati(k,3));
            % stati(k,2) = stati(k,2) + vel(k,1) * dt * sin(stati(k,3));
            % stati(k,3) = stati(k,3) + vel(k,2) * dt;
        end
        

        cla;
        hold on;
        
        if SAVE_TRAJECTORIES
            % Save the updated positions into the trajectories array
            trajectories(:, :, t) = stati(t,1:2);
        end
        
        % Plot the trajectory for each drone up to the current time
        for i = 1:numDrones
            if PLOT_TRAJECTORIES
                % Extract the trajectory so far (squeeze the slice into a 2D array)
                traj = squeeze(trajectories(i, :, 1:t));
                plot(traj(1, :), traj(2, :), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
            end
            % Plot the current drone position as a marker

            drawUnicycle(stati(i,1),stati(i,2),stati(i,3));

            % rotated_vertices = create_triangles(stati(i,:));
            % plot(rotated_vertices(:,1), rotated_vertices(:,2), '-', 'Color', colors(i,:), 'LineWidth', 2);

            % plot(stati(i, 1), stati(i, 2), 'o', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
            
        end
        voronoi(stati(:,1),stati(:,2));
        
        plot(x_fire1,y_fire1,'x','Color', 'r', 'MarkerSize', sigma_fire1)
        plot(x_fire2,y_fire2,'x','Color', 'r', 'MarkerSize', sigma_fire2)
        plot(x_water,y_water,'o','Color', 'b', 'MarkerSize', sigma_water)

        drawnow;  % Force MATLAB to update the figure
        % Optionally add a pause (e.g., pause(0.01)) to slow down the simulation for visualization
    end
end