clear all;
close all;
clc;

%% Simulation Parameters 

dt = 0.01;
T_sim = 5;
scenario = 1;
tot_iter = (T_sim-1)/dt + 1;

DO_SIMULATION = true;
PLOT_DENSITY_FUNCTIONS = false;
PLOT_TRAJECTORIES = false;
PLOT_ITERATIVE_SIMULATION = false;
PLOT_CONSENSUS = false;
ANIMATION = true;

if DO_SIMULATION == true
    ANIMATION = true;
end

%% Vehicles Parameters 

vel_lin_max = 100 ;  % Maximum linear velocity [m/s]
vel_lin_min = 60 ;   % Minimum linear velocity [m/s]
vel_lin_z_max = 100 ; % Maximum linear velocity along z [m/s]
vel_ang_max = 20 ;  % Maximum angular velocity [rad/s]
dim_UAV = 5;  % Dimension of the UAV
numUAV = 5;   % Number of UAV
Kp_z = 60;  % Proportional gain for the linear velocity along z
Kp = 50;   % Proportional gain for the linear velocity  
Ka = 10;   % Proportional gain for the angular velocity 
Ke = 0;  % Additional gain for the angular velocity 
height_flight = 30;   % Height of flight from the ground 

% Take Off
takeOff = true;
freq_takeOff = 10;
n = 1;

% Refill
refill_time = 20;
count_refill = zeros(numUAV,1);

% Generate random starting positions for each point
x = ones(numUAV,1) * 50;   % Random x coordinates
y = ones(numUAV,1) * 250;  % Random y coordinates
z = ones(numUAV,1) * 5;    % Start from ground
theta = ones(numUAV,1) * (-pi/2);

for i = 1:numUAV
    y(i) = y(i) + 10 * i;
end

states = [x, y, z, theta];

objective = ones(numUAV,1) * 2;     % -> objective = 1 : the UAV is filled with
                                    % water and is going to put out the fire
                                    % -> objective = 2 : the UAV is empty and is
                                    % going to refill
                                    % (we assume every one empty at the beginning)

% Dynamics

fun = @(state, u, deltat) [state(1) + u(1) * cos(state(4)) * deltat, ...
                           state(2) + u(1) * sin(state(4)) * deltat, ...
                           state(3) + u(2) * deltat, ...
                           state(4) + u(3) * deltat];

%% Measurement Parameters

% Measurements frequency [cs]
meas_freq_GPS = 10;
meas_freq_ultr = 2;
meas_freq_gyr = 1;

std_gps = 3; % Standard deviation of the GPS
std_ultrasonic = 2; % Standard deviation of the ultrasonic sensor
std_gyro = 0.5; % Standard deviation of the gyroscope
R = diag([std_gps^2, std_gps^2, std_ultrasonic^2, std_gyro^2]); % Covariance of the measurement noise


%% Kalman Filter Parameters

% Jacobian of the state model
% !!! WE ASSUME THE CONTROL AS INDIPENDENT FROM THE STATE -> we put uncertanty in the control !!!
A = @(u, theta, deltat) [1, 0, 0, -u(1) * sin(theta) * deltat;
                         0, 1, 0,  u(2) * cos(theta) * deltat;
                         0, 0, 1,                           0;
                         0, 0, 0,                           1];

% Matrix of the propagation of the process noise for (x,y,z,theta) 4x4
% We considered the niose as white and related to the uncertanty in the
% control (is our uncertanty in the model)
G = @(theta, deltat) [cos(theta) * deltat,      0,      0;
                      sin(theta) * deltat,      0,      0;
                                        0, deltat,      0;
                                        0,      0, deltat];

% Covariance of the process noise
std_u = [2, 2, 2]; % Uncertainty on the velocity (tang , z) and angular velocity
Q = diag(std_u.^2);


% Initial state (x,y,z,theta)
states_est = (states' + [std_gps * randn(2, numUAV); zeros(1, numUAV); std_gyro * randn(1, numUAV)])';

% Observation matrix (H)
H = [1,0,0,    0;
     0,1,0,    0;
     0,0,1,    0;
     0,0,0, 1/dt]; % ( me measure theta with a gyroscope, it measure
                %   the angular velocity so we have
                %   to multiply it by the time step )


% Covariance matrix of the initial estimate
P = eye(4) * 100; % We consider some starting uncertanty

% Map Parameters
dimgrid = [500 500 500];   % Define the dimensions of the grid


%% Fires Parameters

% Fires Positions
x_fire1 = 300;
y_fire1 = 400;
x_fire2 = 450;
y_fire2 = 50;

pos_fire1_start = [x_fire1 , y_fire1];
sigma_fire1_start = 40;   % Standard deviation of the first fire
                          % (correspond to the extention of the fire)

pos_fire1_mov = @(s) [x_fire1 - 5 * (s - 1) , y_fire1 - 2 * (s - 1)]; % t start from 1
sigma_fire1_mov = @(t) 40 - 1 * (t - 1);


pos_fire2 = [x_fire2, y_fire2];
sigma_fire2 = 15;   % Standard deviation of the second fire
                    % (correspond to the extention of the fire)

inc_threshold1 = sigma_fire1_start*0.5;  % Distance that has to be reach from the fire 1 
inc_threshold2 = sigma_fire2*0.5;  % Distance that has to be reach from the fire 2

pos_est_fire1 = zeros(numUAV,2);
sigma_est_fire1 = zeros(numUAV,1);

for i = 1:numUAV
    pos_est_fire1(i,:) = pos_fire1_mov(1);          % Initialize the estimated positions of fire 1
    sigma_est_fire1(i,1) = sigma_fire1_mov(1);      % Initialize the estimated extension of fire 1
end

%% Water Parameters

% Water Positions
x_water = 50;
y_water = 50;

pos_water = [x_water, y_water];

sigma_water = 60;
wat_threshold = 20;   % Distance that has to be reach from the water source to refill

% Density Functions for the fires and the water
[G_fire,G_water] = objective_density_functions(dimgrid, pos_fire1_mov, pos_fire2, pos_water, sigma_fire1_start, sigma_fire2, sigma_water, 0, PLOT_DENSITY_FUNCTIONS);

[Xf, Yf, Zf] = plot_enviroment_surface();


%% Consensus Parameters

sensor_range = 70; % distanza misura infrarossi
meas_fire1 = zeros(numUAV,1);

% Each drone has an estimate of the measurement time of the other drones (we add some uncertanty)
LastMeas = ones(numUAV,numUAV) + 6 * rand(numUAV,numUAV) - 3;           % At the beginning no one UAV has done a measurement

invLastMeas = ones(numUAV,numUAV);
invSumLastMeas = ones(1,numUAV);

Qc = ones(numUAV) * 1/numUAV;                   % Initialization of matrix Q
 

%% Save Matrices Declaration

% Real Trajectories
trajectories = zeros(numUAV, 4, (T_sim-1)/dt);

% Estimated Trajectories
trajectories_est = zeros(numUAV, 4, (T_sim-1)/dt);

% Save all the traces of P
P_trace = zeros(numUAV,(T_sim-1)/dt);

% Centroids Trajectories
centroids_est_stor = zeros(numUAV, 2, (T_sim-1)/dt);

% Estimated Trajectory of X coordinate of fire 1
posFir1StoreX = zeros(numUAV, 1, (T_sim-1)/dt); 
posFir1StoreX(:,1,1) = pos_est_fire1(:,1);

% Estimated Trajectory of Y coordinate of fire 1
posFir1StoreY = zeros(numUAV, 1, (T_sim-1)/dt);
posFir1StoreY(:,1,1) = pos_est_fire1(:,2);

% Behavior of the extension of fire 1
sigmaFir1Stor = zeros(numUAV, 1, (T_sim-1)/dt); 
sigmaFir1Stor(:,1,1) = sigma_est_fire1(:,1);

% Real Path of Fire 1
posFir1StoreReal = zeros(1, 2, (T_sim-1)/dt);
sigmaFir1StoreReal = zeros(1, (T_sim-1)/dt);

% Voronoi Edges
vx_Data = cell(1, tot_iter); % Celle per memorizzare i dati di vx
vy_Data = cell(1, tot_iter); % Celle per memorizzare i dati di vy


%% Simulation
if DO_SIMULATION
    
    count = 0;
    for t = 1:dt:T_sim

        count = count + 1;
        LastMeas = LastMeas + 1;

        posFir1StoreReal(1,:,count) = pos_fire1_mov(t);
        sigmaFir1StoreReal(1,count) = sigma_fire1_mov(t);

        % Compute the distances from fires and water source for each drone

        dist_inc1 = zeros(numUAV,1); 
        dist_real_inc1 = zeros(numUAV,1);
    
        dist_inc2 = pdist2(pos_fire2, states_est(:,1:2)); % Distance to the second fire
        dist_wat  = pdist2(pos_water, states_est(:,1:2)); % Distance to the water source
       
        % If the distatnce bettween the UAV and the Real fire is small, the
        % sensors see the new position and extension

        dist_real_inc1(:,1) = pdist2(pos_fire1_mov(t),states(:,1:2)); % Here we use the real posititon since we are 
                                                                      % considering if the sensor are able to detect the fire

        % Verify if the wanted distance from the target is reached
        for i = 1:numUAV

            dist_inc1(i) = pdist2(pos_est_fire1(i,:), states_est(i,1:2));
            inc_threshold1(i) = sigma_est_fire1(i,1)*0.5;

            if dist_real_inc1(i) <= sensor_range && objective(i) == 1 && meas_fire1(i) ~= 1
                
                % Meaurement
                pos_est_fire1(i,:) = pos_fire1_mov(t) + 10 * rand(1,1) - 5;
                sigma_est_fire1(i,1) = sigma_fire1_mov(t) + 4 * rand(1,1) - 2;

                LastMeas(i,:) = 1 + 5 * rand(1,numUAV);  
                invLastMeas = 1 ./ LastMeas;

                for j = 1:numUAV

                    invSumLastMeas(j) = sum(invLastMeas(:,j));

                    for k = 1:numUAV

                        Qc(j,k) = (invLastMeas(k,j)) / ( invSumLastMeas(j) );

                    end

                end

                meas_fire1(i) = 1;

            end
            
            % If the drone is close to a fire and its objective is 1 (heading to fire)
            if dist_inc1(i) <= inc_threshold1(i) && objective(i) == 1

                objective(i) = 2; % Change objective to 2 (heading to refill water)
                meas_fire1(i) = 0;
            elseif dist_inc2(i) <= inc_threshold2 && objective(i) == 1

                objective(i) = 2; % Change objective to 2 (heading to refill water)
                meas_fire1(i) = 0;
            end

            % If the drone is close to the water source and its objective is 2 (heading to refill)
            if dist_wat(i) <= wat_threshold && objective(i) == 2 && count_refill(i) == 0 

                count_refill(i) = refill_time;

            end

        end

        % Consensus algorithm
        % disp(Qc);
        % We use the same matrix Q for both the coordinates and the extension

        pos_est_fire1(:,1) = Qc * pos_est_fire1(:,1);
        pos_est_fire1(:,2) = Qc * pos_est_fire1(:,2);
        sigma_est_fire1(:,1) = Qc * sigma_est_fire1(:,1);

        posFir1StoreX(:,1,count+1) = pos_est_fire1(:,1);
        posFir1StoreY(:,1,count+1) = pos_est_fire1(:,2);
        sigmaFir1Stor(:,1,count+1) = sigma_est_fire1(:,1);

        % Compute Voronoi tessellation and velocities
        [areas, centroids_est, control_est] = voronoi_function_FW(numUAV, dimgrid, states, Kp_z, Kp, Ka, Ke, pos_est_fire1, pos_fire2, ...
                                                                  sigma_est_fire1, sigma_fire2, G_water, height_flight, scenario, objective);

        
        % Impose a maximum velocity
        % The linear straight velocty has also a minimum velocity since we are considering Fixed wing UAV 
        control_est(:,1) = sign(control_est(:,1)) .* max(min(abs(control_est(:,1)), vel_lin_max), vel_lin_min); % Linear velocity 
        control_est(:,2) = sign(control_est(:,2)) .* min(abs(control_est(:,2)), vel_lin_z_max); % Limit linear velocity along z
        control_est(:,3) = sign(control_est(:,3)) .* min(abs(control_est(:,3)), vel_ang_max); % Limit angular velocity

        if takeOff && n ~= numUAV + 1

            for s = n:numUAV
                control_est(s,:) = [0,0,0];
            end

            if mod(count, freq_takeOff) == 0
                n = n + 1;
            end

        end

        for k = 1:numUAV

            % Refill control
            if count_refill(k) ~= 0
                control_est(k,1) = vel_lin_min ;
                control_est(k,3) = 0;
                count_refill(k) = count_refill(k) - 1;
            end

            if count_refill(k) == 0 && dist_wat(k) <= wat_threshold

                objective(k) = 1; % Change objective to 1 (heading to fire)
                meas_fire1(k) = 0;

            end

            centroids_est_stor(k,:,count) = centroids_est(k,:);

            % Model Simulation - REAL 
            states(k,:) = fun(states(k,:), control_est(k,:), dt);    % we use to control the real model the velocities computed using
                                                                     % the estimated states (as it will be in real applications)
            trajectories(k,:,count) = states(k,:);

        end

        % Extended Kalman Filter
        measure = (H * states' + [std_gps * randn(2, numUAV); ...
                                  std_ultrasonic * randn(1, numUAV); ...
                                  std_gyro * randn(1, numUAV)])';

        for k = 1:numUAV

            [states_est(k,:), P] = ExtendedKalmanFilter_function(states_est(k,:), ...
                measure(k,:), control_est(k,:), A, G, fun, Q, H, R, P, count, ...
                meas_freq_GPS, meas_freq_ultr, meas_freq_gyr, dt);

            P_trace(k,count) = trace(P);

            trajectories_est(k,:,count) = states_est(k,:);
        end

        % Save Voronoi edges
        [vx, vy] = voronoi(trajectories_est(:,1,count), trajectories_est(:,2,count));
        vx_Data{count} = vx;
        vy_Data{count} = vy;

        %% Plots
        % real and estimated   
        if PLOT_ITERATIVE_SIMULATION

            curr_fire1_pos = pos_fire1_mov(t);
            curr_fire1_sig = sigma_fire1_mov(t);
            plotSimulation_function(states, states_est, centroids_est, numUAV, dimgrid, pos_est_fire1, curr_fire1_pos(1), ...
                                    curr_fire1_pos(2), sigma_est_fire1, curr_fire1_sig, x_fire2, y_fire2, sigma_fire2, ...
                                    x_water, y_water, sigma_water, Xf, Yf, Zf, dim_UAV, 3);

        end

        fprintf('Iteration n: %d / %d\n', count, tot_iter);
        
    end

    if PLOT_TRAJECTORIES
        plotUAVTrajectories_function(numUAV, trajectories, trajectories_est, P_trace);
    end

    if PLOT_CONSENSUS 
        plotConsensus_function(numUAV, posFir1StoreReal, posFir1StoreX, posFir1StoreY, sigmaFir1StoreReal, sigmaFir1Stor);  
    end

end

if ANIMATION 
    %% Plot 2D voronoi simulation
    figure(50)
    bx = subplot(1,1,1);
    axis(bx,[0 dimgrid(1) 0 dimgrid(2)]);
    xlabel(bx,'X Coordinate');
    ylabel(bx,'Y Coordinate');
    view(bx,2);
    

     for t = 1:count
    
        cla(bx);
        hold(bx,'on');

        plot(bx,posFir1StoreReal(1,1,t), posFir1StoreReal(1,2,t), 'rx','MarkerSize', sigmaFir1StoreReal(1,t));
        plot(bx,x_fire2,y_fire2,'x','Color', 'r', 'MarkerSize', sigma_fire2)
        plot(bx,x_water,y_water,'o','Color', 'b', 'MarkerSize', sigma_water)
        for i = 1:numUAV
            % Draw the UAV pose
            drawUAV2D(trajectories(i, 1,t), trajectories(i, 2,t), trajectories(i, 4,t), dim_UAV,'k');
            drawUAV2D(trajectories_est(i, 1,t), trajectories_est(i, 2,t), trajectories_est(i, 4,t), dim_UAV,'g');
    
            plot(bx,centroids_est_stor(i,1,t), centroids_est_stor(i,2,t), 'x', 'Color', 'g');
            plot(bx,posFir1StoreX(i,1,t), posFir1StoreY(i,1,t), 'x','MarkerSize', sigmaFir1Stor(i,1,t));
        end
    
        %[vx_es, vy_es] = voronoi(trajectories_est(:,1,t), trajectories_est(:,2,t)); 
        vx_es = vx_Data{t};
        vy_es = vy_Data{t};
        plot(bx,vx_es, vy_es, 'g-');
    
        hold(bx,'off');
        drawnow; 
        
    end  



    %% Plot 3D Simulation
    figure(60);
    ax = subplot(1,1,1);
    axis(ax,[0 dimgrid(1) 0 dimgrid(2) 0 dimgrid(3)]);
    hold(ax,'on');
    
    % Carica la texture
    texture = imread('Mountain.jpg');  % deve essere RGB
    texture_resized = imresize(texture, [size(Xf,1), size(Xf,2)]);
    texture_resized = double(texture_resized) / 255;  % Normalizza tra 0 e 1

    % Aumenta la luminosità
    brightness_factor = 1.5;
    texture_resized = min(texture_resized * brightness_factor, 1);

    % Crea uno sfondo verde uniforme
    green_background = repmat(reshape([0.4660 0.6740 0.1880], 1, 1, 3), size(texture_resized, 1), size(texture_resized, 2));

    % Fattore di trasparenza (0 = solo sfondo verde, 1 = solo texture)
    alpha = 0.6;  % <-- regola questo valore

    % Mix tra texture e sfondo
    blended_texture = alpha * texture_resized + (1 - alpha) * green_background;

    % ─── static terrain ───────────────────────────────
    surf(ax, Xf, Yf, Zf, 'CData', flip(blended_texture, 1), ...
        'FaceColor', 'texturemap', ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'none');

    lighting none;


    % Aggiungi le linee di contorno
    contour3(ax, Xf, Yf, Zf, 20, 'k');
    
    % ─── static water circle ──────────────────────────
    theta = linspace(0,2*pi,100);
    r = sigma_water/2;
    x_circle = r*cos(theta)+x_water;
    y_circle = r*sin(theta)+y_water;
    z_circle = 5 * ones(size(theta));
    hWater = plot3(ax,x_circle, y_circle, z_circle, 'b', 'LineWidth', 2);
    
    % ─── static second fire ──────────────────────────
    hFire2 = plot3(ax,x_fire2, y_fire2, ...
                   enviroment_surface(x_fire2,y_fire2,1), ...
                   'x', 'Color', 'r', ...
                   'MarkerSize', sigma_fire2, 'LineWidth', 2);
    
    view(ax,3);
    grid(ax,'on');
    xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');

    
    % ——— define a simple airplane in its body‑frame ———
    fuselage = [  1;  0;  0 ];        % nose
    wingL    = [ 0;  0.5;  0 ];      % left wingtip
    wingR    = [ 0; -0.5;  0 ];      % right wingtip
    tailT    = [ -0.5; 0;   0.2 ];    % tail top
    tailB    = [ -0.5; 0;  -0.2 ];    % tail bottom
    
    % vertices matrix
    V = [ fuselage, wingL, wingR, tailT, tailB ]';   % 5×3
    
    % faces list (triangles)
    F = [ ...
        1 2 3;   % main wing
        1 4 5;   % tailplane top/bottom
        2 4 3;   % left side fuselage
        3 5 2;   % right side fuselage
    ];
    
    V = dim_UAV * V;
    
    hHeightLine = gobjects(numUAV,1);
    
    % number of UAVs
    hPlaneTF = gobjects(numUAV,1);
    for i = 1:numUAV
       
        hPlaneTF(i) = hgtransform;
    
        patch(ax, 'Vertices', V, 'Faces', F, ...
               'FaceColor', rand(1,3)*0.5 + 0.5, ...
               'EdgeColor', 'k', ...
               'Parent', hPlaneTF(i) );
    
        hHeightLine(i) = plot3(ax,[NaN NaN], [NaN NaN], [NaN NaN], ...  
                                ':', ...      
                                'Color', [0 0 0], ...
                                'LineWidth', 0.2 );
    
    end
    
    % placeholders for the real‐fire and estimated‐fire markers
    hFireReal = plot3(ax,NaN, NaN, NaN, 'xr', 'LineWidth', 2);
    hFireEst  = plot3(ax,NaN, NaN, NaN, 'x',  'MarkerSize', 8);
    
    
    for t = 1:count
        % ─── update each UAV’s position ──────────────────────
        for i = 1:numUAV
    
            x = trajectories(i,1,t);
            y = trajectories(i,2,t);
            z = trajectories(i,3,t);
            theta = trajectories(i,4,t); 
    
            % surface height under the UAV    
            z0  = enviroment_surface(x, y, 1);
    
            T = makehgtform('translate',[x,y,z], ...
                            'zrotate', theta);
    
            set(hPlaneTF(i), 'Matrix', T);
    
            set(hHeightLine(i), ...
                'XData', [x,    x], ...
                'YData', [y,    y], ...
                'ZData', [z0,   z] );
        end
    
        % ─── update the “real” fire marker ────────────────────
        set(hFireReal, ...
            'XData', posFir1StoreReal(1,1,t), ...
            'YData', posFir1StoreReal(1,2,t), ...
            'ZData', enviroment_surface(...
                       posFir1StoreReal(1,1,t), ...
                       posFir1StoreReal(1,2,t), 1), ...
            'MarkerSize', sigmaFir1StoreReal(1,t));
    
        % ─── update the “estimated” fire marker ───────────────
        set(hFireEst, ...
            'XData', posFir1StoreX(i,1,t), ...
            'YData', posFir1StoreY(i,1,t), ...
            'ZData', enviroment_surface(...
                       posFir1StoreX(i,1,t), ...
                       posFir1StoreY(i,1,t), 1), ...
            'MarkerSize', sigmaFir1Stor(i,1,t));

        hold(ax,'off');
        drawnow; 
        
    end 

end

   