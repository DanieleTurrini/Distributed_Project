clear all;
close all;
clc;

%% Parameters definition

DO_SIMULATION = true;
INITIAL_VALUES = false;
PLOT_TRAJECTORIES = true;
SAVE_TRAJECTORIES = true;

% Define the number of points and grid dimensions
numDrones = 5;      % Set the number of drones
dimgrid = [500 500 500];   % Define the height of the grid
acc_max = 80;  % Maximum velocity of the drones
inc_threshold1 = 40;    % Distance that has to be reach from fire to throw the water
inc_threshold2 = 40;    % Distance that has to be reach from fire to throw the water
wat_threshold = 20;     % Distance that has to be reach from center of the lake to refill
dev_std_gps = 1;        % GPS uncertanty

T_sim = 100;
dt = 0.01;

Kp = 20;                % Lyoid proportional gain
Ki = 0.1;               % Guadagno integrale
Kd = 0.05;              % Guadagno derivativo

% Matrice di transizione di stato
A = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1];

% Matrice di controllo 
B = [0.5 * dt^2,          0,          0;
              0, 0.5 * dt^2,          0;
              0,          0, 0.5 * dt^2;
             dt,          0,          0;
              0,         dt,          0
              0,          0,         dt;];

%% Fires positions function

% Fires Positions
x_fire1 = 400;
y_fire1 = 400;
z_fire1 = 0;

x_fire2 = 450;
y_fire2 = 50;
z_fire2 = 0;

% Water Positions
x_water = 50;
y_water = 50;
z_water = 0;

pos_fire = [x_fire1, y_fire1, z_fire1;
            x_fire2, y_fire2, z_fire2];
pos_water = [x_water, y_water,z_water];

% Standard deviations of fires and water source
sigma_fire = [30,15];
sigma_water = 20;

% Creazione della griglia di punti
[x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));

% Calcolo della distribuzione gaussiana per incendi e acqua
G_fire = zeros(size(x_m));
for i = 1:size(pos_fire, 1)
    G_fire = G_fire + exp(-(((x_m - pos_fire(i, 1)).^2) / (2 * sigma_fire(i)^2) + ((y_m - pos_fire(i, 2)).^2) / (2 * sigma_fire(i)^2)));
end
G_water = exp(-(((x_m - pos_water(1)).^2) / (2 * sigma_water^2) + ((y_m - pos_water(2)).^2) / (2 * sigma_water^2)));

%% Creazione dei droni
% Punti iniziali
start_points = zeros(numDrones,3);
start_points(:,1:2) = rand(numDrones, 2) * 100; % Posizioni casuali in un'area 100x100

droni = creaDroni(numDrones, start_points, dev_std_gps); 

% Visualizzazione in 3D
figure(1);
surf(x_m, y_m, G_fire);
shading interp; % Per rendere la superficie più liscia
colormap jet;
colorbar;
xlabel('X');
ylabel('Y');
zlabel('Densità');
title('Funzione densità: Incendi');
drawnow;


%% Simulation

trajectories = zeros(numDrones,3,T_sim/dt);
curr_real_pos = zeros(numDrones,3);
if DO_SIMULATION

    for i = 1:numDrones
        
        if SAVE_TRAJECTORIES
            trajectories(i,:,1) = droni(i).pos_reale;
        end

        curr_real_pos(i,:) = droni(i).pos_reale;
        curr_real_state(:,i) = droni(i).state;
    end
    
    
    for t = 2:dt:T_sim

        dist_inc1 = pdist2(pos_fire(1,:), curr_real_pos);
        dist_inc2 = pdist2(pos_fire(2,:), curr_real_pos);
        dist_wat = pdist2(pos_water, curr_real_pos);

        for i= 1:numDrones
            if(dist_inc1(i) <= inc_threshold1 || dist_inc2(i) <= inc_threshold2)
                droni(i).objective = 2;
            end
            if dist_wat(i) <= wat_threshold 
                droni(i).objective = 1;
            end
        end

        [areas, weigth_centroids, acc_des] = voronoi_function_3D(droni, dimgrid, Kp, Ki, Kd, dt, G_fire, G_water);
        
        % Impose a maximum acceleration
        acc = sign(acc_des) .* min(abs(acc_des), acc_max);
        for k = 1:numDrones
            curr_real_pos(k,:) = A * droni(k).state + B * acc(k,:)';

            if SAVE_TRAJECTORIES
                trajectories(k,:,1) = curr_real_pos(k,:);
            end
        end

    end

    for i = 1:numDrones
        if PLOT_TRAJECTORIES
            % Extract the trajectory so far (squeeze the slice into a 2D array)
            traj = squeeze(trajectories(i, :, 1:t));
            plot(traj(1, :), traj(2, :), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
        end
        % Plot the current drone position as a marker
        plot(nx(i, 1), nx(i, 2), 'o', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
        
    end
end