clear;
close all;
clc;

%% Simulation Parameters 

dt = 0.01;
T_sim = 15;
scenario = 1;
tot_iter = (T_sim - 1)/dt + 1;


DO_SIMULATION = true;
UAV_FAIL = true;
PLOT_DENSITY_FUNCTIONS = false;
PLOT_TRAJECTORIES = false;
PLOT_COVARIANCE_TRACE = false;
PLOT_CONSENSUS = true;

PLOT_ITERATIVE_SIMULATION = false;
ANIMATION = true;

if ANIMATION == true
    DO_SIMULATION = true;
end

%% Vehicles Parameters 

vel_lin_max = 100 ;                 % Maximum linear velocity [m/s]
vel_lin_min = 70 ;                  % Minimum linear velocity [m/s]
vel_lin_z_max = 100 ;               % Maximum linear velocity along z [m/s]
vel_ang_max = 20 ;                  % Maximum angular velocity [rad/s]
dim_UAV = 5;                        % Dimension of the UAV
numUAV = 5;                         % Number of UAV
Kp_z = 100;                         % Proportional gain for the linear velocity along z
Kp = 50;                            % Proportional gain for the linear velocity  
Ka = 10;                            % Proportional gain for the angular velocity
height_flight = 30;                 % Height of flight from the ground 

% Take Off
takeOff = true;
freq_takeOff = 30;                  % Time distance between each takeoff 
n = 1;

% Refill
refill_time = 30;                   % Time needed to do the refill
count_refill = zeros(numUAV,1);

% Starting points
x = ones(numUAV,1) * 50;   % Random x coordinates
y = ones(numUAV,1) * 250;  % Random y coordinates
z = ones(numUAV,1);    % Start from ground
theta = ones(numUAV,1) * (- pi/2);

for i = 1:numUAV
    y(i) = y(i) + 30 * i;
    z(i) = enviroment_surface(x(i),y(i),scenario) + 0.2;
end

% initial positions of uav
initialUAV_pos = [x, y, z, theta];

states = [x, y, z, theta];

objective = ones(numUAV,1) * 2;     % -> objective = 1 : the UAV is filled with
                                    % water and is going to put out the fire
                                    % -> objective = 2 : the UAV is empty and is
                                    % going to refill
                                    % -> objective = 3 : all fires estinguished
                                    % (we assume every one empty at the beginning)

% Dynamics

fun = @(state, u, deltat) [state(1) + u(1) * cos(state(4)) * deltat, ...
                           state(2) + u(1) * sin(state(4)) * deltat, ...
                           state(3) + u(2) * deltat, ...
                           state(4) + u(3) * deltat];

%% UAV Fail paramters
UAV_check_fail = false; % Check if the UAV is failed
fail_time = 8; % Time instant when one UAV fail 
ind = 3; % UAV that fails
check = ones(numUAV, 1); % they periodically exchange the check 
check_treshold = 10; % if the check of that UAV is 1 for 10 times, it is
                        % considered failed 
check_count = zeros(numUAV, 1);
communication_prob = 0.05; % Probability of NOT communication

%% Measurement Parameters

% Measurements frequency [cs]
meas_freq_GPS = 20; % 5 Hz
meas_freq_ultr = 4; % 25 Hz
meas_freq_gyr = 1; % 50 Hz

std_gps = 3; % Standard deviation of the GPS
std_ultrasonic = 2; % Standard deviation of the ultrasonic sensor
std_gyro = 1; % Standard deviation of the gyroscope
R = diag([std_gps^2, std_gps^2, std_ultrasonic^2, std_gyro^2]); % Covariance of the measurement noise

%% Kalman Filter Parameters

% Jacobian of the state model
% !!! WE ASSUME THE CONTROL AS INDIPENDENT FROM THE STATE -> we put uncertanty in the control !!!
A = @(u, theta, deltat) [1, 0, 0, -u(1) * sin(theta) * deltat;
                         0, 1, 0,  u(1) * cos(theta) * deltat;
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
std_u = [2, 2, 2]; % Uncertainty on the velocity linear in x-y , linear in z and angular velocity
Q = diag(std_u.^2);


% Initial state (x,y,z,theta)
states_est = (states' + [std_gps * randn(2, numUAV); zeros(1, numUAV); std_gyro * randn(1, numUAV)])';

% Observation matrix (H)
H = [1,0,0,    0;
     0,1,0,    0;
     0,0,1,    0;
     0,0,0, 1/dt];  % ( me measure theta with a gyroscope, it measure
                    %   the angular velocity so we have
                    %   to multiply it by the time step )

% Covariance matrix of the initial estimate
P = eye(4) * 30; % We consider some starting uncertanty

% Map Parameters
dimgrid = [500 500 500];   % Define the dimensions of the grid


%% Fires Parameters

drop_dist = 0.6; % Percentage of the distance from the center of 
                 % the fire that has to be reached until drop the water 

% Fires Positions
x_fire1 = 300;
y_fire1 = 400;
x_fire2 = 450;
y_fire2 = 50;

pos_fire1_start = [x_fire1 , y_fire1];
pos_fire2_start = [x_fire2 , y_fire2];
% sigma_fire1_start = 40;   % Standard deviation of the first fire
                          % (correspond to the extention of the fire)

pos_fire1_mov = @(t) [x_fire1 - 5 * (t - 1) , y_fire1 - 2 * (t - 1)]; % t start from 1
% sigma_fire1_mov = @(t) 40 - 1 * (t - 1);
sigma_fire1 = 50;   % Standard deviation of the first fire
                    % (correspond to the extention of the fire)


pos_fire2_mov = @(t) [x_fire2 - 2.5 * (t - 1) , y_fire2 + 1 * (t - 1)]; % t start from 1
sigma_fire2 = 20;   % Standard deviation of the second fire
                    % (correspond to the extention of the fire)

inc_threshold1 = sigma_fire1 * drop_dist;  % Distance that has to be reach from the fire 1 
inc_threshold2 = sigma_fire2 * drop_dist;  % Distance that has to be reach from the fire 2

pos_est_fire1 = zeros(numUAV,2);
sigma_est_fire1 = zeros(numUAV,1);
pos_est_fire2 = zeros(numUAV,2);
sigma_est_fire2 = zeros(numUAV,1);

for i = 1:numUAV
    % --- Fire 1 ---
    pos_est_fire1(i,:) = pos_fire1_mov(1);          % Initialize the estimated positions of fire 1
    %sigma_est_fire1(i,1) = sigma_fire1_mov(1);      % Initialize the estimated extension of fire 1
    sigma_est_fire1(i,1) = sigma_fire1;

    % --- Fire 2 ---
    pos_est_fire2(i,:) = pos_fire2_mov(1);                 % Initialize the estimated positions of fire 2
    sigma_est_fire2(i,1) = sigma_fire2;             % Initialize the estimated extension of fire 2
end

% Decreasing factor of the fire
deacreasingFire_factor = 8;    % Decreasing factor of the fire extension
                                % (we assume that the fire decrease every time the UAV drop the water)

%% Water Parameters

% Water Positions
x_water = 50;
y_water = 50;

pos_water = [x_water, y_water];

sigma_water = 40;
wat_threshold = 30;   % Distance that has to be reach from the water source to refill

% Density Functions for the fires and the water
[G_fire,G_water] = objective_density_functions(dimgrid, pos_fire1_mov, pos_fire2_mov, pos_water, sigma_fire1, sigma_fire2, sigma_water, 0, PLOT_DENSITY_FUNCTIONS);

[Xf, Yf, Zf] = plot_enviroment_surface(false);


%% Consensus Parameters

sensor_range = 70; % Infrared measurement distance
meas_fire1 = zeros(numUAV,1);
meas_fire2 = zeros(numUAV,1);

% Each drone has an estimate of the measurement time of the other drones (we add some uncertanty)
LastMeas1 = ones(numUAV,numUAV) + 6 * rand(numUAV,numUAV) - 3;           % At the beginning no one UAV has done a measurement
LastMeas2 = ones(numUAV,numUAV) + 6 * rand(numUAV,numUAV) - 3;           % At the beginning no one UAV has done a measurement

invSumLastMeas1 = ones(1,numUAV);
invSumLastMeas2 = ones(1,numUAV);

Qc1 = ones(numUAV) * 1/numUAV;                   % Initialization of matrix Q for fire 1
Qc2 = ones(numUAV) * 1/numUAV;                   % Initialization of matrix Q for fire 2
 

%% Save Matrices Declaration

% Real Trajectories
trajectories = zeros(numUAV, 4, (T_sim-1)/dt+1);

% Estimated Trajectories
trajectories_est = zeros(numUAV, 4, (T_sim-1)/dt+1);

% Save all the traces of P
P_trace = zeros(numUAV,(T_sim-1)/dt+1);

% Centroids Trajectories
centroids_est_stor = zeros(numUAV, 2, (T_sim-1)/dt+1);


Fir1Store = zeros(numUAV, 3, (T_sim-1)/dt+1); 
% Estimated Trajectory of X coordinate of fire 1
Fir1Store(:,1,1) = pos_est_fire1(:,1);
% Estimated Trajectory of Y coordinate of fire 1
Fir1Store(:,2,1) = pos_est_fire1(:,2);
% Behavior of the extension of fire 1
Fir1Store(:,3,1) = sigma_est_fire1(:,1);


Fir2Store = zeros(numUAV, 3, (T_sim-1)/dt+1);
% Estimated Trajectory of X coordinate of fire 2
Fir2Store(:,1,1) = pos_est_fire2(:,1);
% Estimated Trajectory of Y coordinate of fire 2
Fir2Store(:,2,1) = pos_est_fire2(:,2);
% Behavior of the extension of fire 2
Fir2Store(:,3,1) = sigma_est_fire2(:,1);

% Real Path of Fire 1
posFir1StoreReal = zeros(1, 2, (T_sim-1)/dt+1);
sigmaFir1StoreReal = zeros(1, (T_sim-1)/dt+1);

% Real Path of Fire 2
posFir2StoreReal = zeros(1, 2, (T_sim-1)/dt+1);
sigmaFir2StoreReal = zeros(1, (T_sim-1)/dt+1);

% Initialization of the distances from fires and water source for each drone
dist_inc1 = zeros(numUAV,1); 
dist_real_inc1 = zeros(numUAV,1);

dist_inc2 = zeros(numUAV,1);
dist_real_inc2 = zeros(numUAV,1);

% Voronoi Edges
vx_Data = cell(1, tot_iter); % Celle per memorizzare i dati di vx
vy_Data = cell(1, tot_iter); % Celle per memorizzare i dati di vy

% Creazione della griglia di punti
[x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));

figure(100);
xlabel('X');
ylabel('Y');
zlabel('Density');
title('Fires density function');

%% Simulation
if DO_SIMULATION
    
    count = 0;
    for t = 1:dt:T_sim

        count = count + 1;
        LastMeas1 = LastMeas1 + 1;
        LastMeas2 = LastMeas2 + 1;

        %% Check commumication
        % See if communication is present
        for k = 1:numUAV
            if rand(1) < communication_prob && count > 1
                check(k) = 0; % NO communication
            else
                check(k) = 1; % YES communication
            end

            if UAV_FAIL && t >= fail_time + dt
                check(ind) = 0;
            end

            if check(k) == 0 % NO communication
                check_count(k) = check_count(k) + 1;

                % Set the previous position and fire estimation
                states_est(k,:) = trajectories_est(k,:,count-1);
                pos_est_fire1(k,:) = Fir1Store(k,1:2,count-1);
                pos_est_fire2(k,:) = Fir2Store(k,1:2,count-1);
                sigma_est_fire1(k,1) = Fir1Store(k,3,count-1);
                sigma_est_fire2(k,1) = Fir2Store(k,3,count-1);



            else
                check_count(k) = 0;
            end
            
            if check_count(k) >= check_treshold
                UAV_check_fail = true;
                ind_est = k;
            end
        end 

        % Real fire position and extension

        % Fire 1
        posFir1StoreReal(1,:,count) = pos_fire1_mov(t);
        % sigmaFir1StoreReal(1,count) = sigma_fire1_mov(t);
        sigmaFir1StoreReal(1,count) = sigma_fire1;

        % Fire 2
        posFir2StoreReal(1,:,count) = pos_fire2_mov(t);
        sigmaFir2StoreReal(1,count) = sigma_fire2;

        % compute the distance from the fire and the water source
        % dist_inc2 = pdist2(pos_fire2, states_est(:,1:2)); % Distance to the second fire
        dist_wat  = pdist2(pos_water, states_est(:,1:2)); % Distance to the water source
        
        % If the distatnce between the UAV and the Real fire is small, the
        % sensors see the new position and extension

        dist_real_inc1(:,1) = pdist2(pos_fire1_mov(t),states(:,1:2)); % Here we use the real posititon since we are 
                                                                      % considering if the sensor are able to detect the fire
        dist_real_inc2(:,1) = pdist2(pos_fire2_mov(t),states(:,1:2)); % Here we use the real posititon since we are 
                                                                      % considering if the sensor are able to detect the fire

        % Verify if the wanted distance from the target is reached
        for i = 1:numUAV
            
            dist_inc1(i) = pdist2(pos_est_fire1(i,:), states_est(i,1:2)); % Distance to the first fire
            dist_inc2(i) = pdist2(pos_est_fire2(i,:), states_est(i,1:2)); % Distance to the second fire
            inc_threshold1(i) = sigma_est_fire1(i,1) * drop_dist;
            inc_threshold2(i) = sigma_est_fire2(i,1) * drop_dist; % Distance that has to be reach from the fire 2

            % --- Fire 1 ---
            if dist_real_inc1(i) <= sensor_range && objective(i) == 1 && meas_fire1(i) ~= 1    
                % Meaurement
                pos_est_fire1(i,:) = pos_fire1_mov(t) + 10 * rand(1,1) - 5;
                %sigma_est_fire1(i,1) = sigma_fire1_mov(t) + 4 * rand(1,1) - 2;
                sigma_est_fire1(i,1) = sigma_fire1 + 4 * rand(1,1) - 2;

                LastMeas1(i,:) = 1 + 2 * rand(1,numUAV);  
                invLastMeas1 = 1 ./ LastMeas1;

                for j = 1:numUAV
                    invSumLastMeas1(j) = sum(invLastMeas1(:,j));
                    for k = 1:numUAV
                        Qc1(j,k) = (invLastMeas1(k,j)) / ( invSumLastMeas1(j) );
                    end
                end
                meas_fire1(i) = 1;
            end

            % --- Fire 2 ---
            if dist_real_inc2(i) <= sensor_range && objective(i) == 1 && meas_fire2(i) ~= 1
                % Meaurement
                pos_est_fire2(i,:) = pos_fire2_mov(t) + 10 * rand(1,1) - 5;
                sigma_est_fire2(i,1) = sigma_fire2 + 4 * rand(1,1) - 2;

                LastMeas2(i,:) = 1 + 2 * rand(1,numUAV);  
                invLastMeas2 = 1 ./ LastMeas2;

                for j = 1:numUAV
                    invSumLastMeas2(j) = sum(invLastMeas2(:,j));
                    for k = 1:numUAV
                        Qc2(j,k) = (invLastMeas2(k,j)) / ( invSumLastMeas2(j) );
                    end
                end
                meas_fire2(i) = 1;
            end
            
            % If the drone is close to a fire and its objective is 1 (heading to fire)
            if dist_inc1(i) <= inc_threshold1(i) && objective(i) == 1
                sigma_fire1 = sigma_fire1 - deacreasingFire_factor;
                if sigma_fire1 <= 0
                    % sigma_fire1 = 0;
                end
                objective(i) = 2; % Change objective to 2 (heading to refill water)
                meas_fire1(i) = 0;

            elseif dist_inc2(i) <= inc_threshold2(i) && objective(i) == 1
                sigma_fire2 = sigma_fire2 - deacreasingFire_factor;
                if sigma_fire2 <= 0
                    % sigma_fire2 = 0;
                end
                objective(i) = 2; % Change objective to 2 (heading to refill water)
                meas_fire1(i) = 0;
            end

            % If the drone is close to the water source and its objective is 2 (heading to refill)
            if dist_wat(i) <= wat_threshold && objective(i) == 2 && count_refill(i) == 0 

                count_refill(i) = refill_time;

            end

            % if all the fires are extinguished, the UAVs objective is = 3 
            if sigma_est_fire1(i,1) <= 0 && sigma_est_fire2(i,1) <= 0
                disp('ALL FIRE ESTINGUISHED');
                objective(i) = 3; % Change objective to 3 (all fires estinguished)
                meas_fire1(i) = 0;
                meas_fire2(i) = 0;
            end

        end

        % disp(objective);

        %% Consensus algorithm
        % We use the same matrix Q for both the coordinates and the extension

        % --- Fire 1 ---
        pos_est_fire1(:,1) = Qc1 * pos_est_fire1(:,1);
        pos_est_fire1(:,2) = Qc1 * pos_est_fire1(:,2);
        sigma_est_fire1(:,1) = Qc1 * sigma_est_fire1(:,1);

        % --- Fire 2 ---
        pos_est_fire2(:,1) = Qc2 * pos_est_fire2(:,1);
        pos_est_fire2(:,2) = Qc2 * pos_est_fire2(:,2);
        sigma_est_fire2(:,1) = Qc2 * sigma_est_fire2(:,1);

        % if the sigma is less than a trashold, we set it to 0
        trashold_sigma_fire = 10;
        if sigma_est_fire1(:,1) < trashold_sigma_fire
            sigma_est_fire1(:,1) = 0;
        end
        if sigma_est_fire2(:,1) < trashold_sigma_fire
            sigma_est_fire2(:,1) = 0;
        end


        % Compute Voronoi tessellation and velocities
        [areas, centroids_est, control_est] = voronoi_function_FW(count,numUAV, dimgrid, states_est, Kp_z, Kp, Ka, pos_est_fire1, pos_est_fire2, ...
                                                                  sigma_est_fire1, sigma_est_fire2, G_water, height_flight, scenario, objective,...
                                                                  initialUAV_pos);
                                                                  
        
        % Impose a maximum velocity
        % The linear straight velocty has also a minimum velocity since we are considering Fixed wing UAV 
        control_est(:,1) = sign(control_est(:,1)) .* max(min(abs(control_est(:,1)), vel_lin_max), vel_lin_min); % Linear velocity 
        control_est(:,2) = sign(control_est(:,2)) .* min(abs(control_est(:,2)), vel_lin_z_max); % Limit linear velocity along z
        control_est(:,3) = sign(control_est(:,3)) .* min(abs(control_est(:,3)), vel_ang_max); % Limit angular velocity

        % If the UAV broke
        if UAV_FAIL && t >= fail_time + dt && UAV_check_fail == false
            control_est(ind,:) = [0,0,0];
        end 


        %% Landing control
        for i = 1:numUAV
            dist_to_initial = norm(states(i,1:2) - initialUAV_pos(i,1:2));
            if dist_to_initial < 100 && objective(i) == 3
                % If the drone is close to the initial position, set the vertical speed
                %control_est(i,1) = min(vel_lin_min, Kp * dist_to_initial);
                control_est(i,2) = sign( 0.5 - states(i,3)) * min(vel_lin_z_max, Kp_z/100 * abs( 0.5 - states(i,3))); % flight_surface(initialUAV_pos(i,1),initialUAV_pos(i,1),0,1) +
                %control_est(i,3) = Ka * ( initialUAV_pos(i,4) - states(i,4) );

                if dist_to_initial < 5
                    control_est(i,1:2) = 0;
                    % control_est(i,3) = sign(initialUAV_pos(i,4) - states(i,4)) * min(vel_ang_max, Ka * abs(initialUAV_pos(i,4) - states(i,4)));
                end

            end
        end 



        %% TakeOff 
        if takeOff && n ~= numUAV + 1

            for s = n:numUAV
                control_est(s,:) = [0,0,0];
            end

            if mod(count, freq_takeOff) == 0
                n = n + 1;
            end

        end

        %% Refill control
        for k = 1:numUAV

            
            if count_refill(k) ~= 0
                control_est(k,1) = vel_lin_min ;
                control_est(k,3) = 0;
                count_refill(k) = count_refill(k) - 1;
            end

            if count_refill(k) == 0 && dist_wat(k) <= wat_threshold

                % if fires are extinguished, the UAVs objective is = 3
                if sigma_est_fire1(k,1) <= 0 && sigma_est_fire2(k,1) <= 0
                    objective(k) = 3; % Change objective to 3 (all fires estinguished)
                    meas_fire1(k) = 0;
                    meas_fire2(k) = 0;
                else
                    objective(k) = 1; % Change objective to 1 (heading to fire)
                    meas_fire1(k) = 0;
                    meas_fire2(k) = 0;
                end


            end

            

            %% Model Simulation - REAL 
            states(k,:) = fun(states(k,:), control_est(k,:), dt);    % we use to control the real model the velocities computed using
                                                                     % the estimated states (as it will be in real applications)
           
        end

        
        %% Extended Kalman Filter
        measure = (H * states' + [std_gps * randn(2, numUAV); ...
                                  std_ultrasonic * randn(1, numUAV); ...
                                  std_gyro * randn(1, numUAV)])';

        for k = 1:numUAV

            [states_est(k,:), P] = ExtendedKalmanFilter_function(states_est(k,:), ...
                                            measure(k,:), control_est(k,:), A, G, fun, Q, H, R, P, count, ...
                                            meas_freq_GPS, meas_freq_ultr, meas_freq_gyr, dt);

            P_trace(k,count) = trace(P);
        end

        % Save Voronoi edges
        [vx, vy] = voronoi(states_est(:,1), states_est(:,2));
        vx_Data{count} = vx;
        vy_Data{count} = vy;

        %% UAV fail 
        for k = 1:numUAV

            % Storing data properly during UAV fail
            if UAV_FAIL && t >= fail_time + dt
                if k < ind

                    trajectories(k,:,count) = states(k,:);
                    trajectories_est(k,:,count) = states_est(k,:);
                    centroids_est_stor(k,:,count) = centroids_est(k,:);

                    Fir1Store(k,1,count) = pos_est_fire1(k,1);
                    Fir1Store(k,2,count) = pos_est_fire1(k,2);
                    Fir1Store(k,3,count) = sigma_est_fire1(k,1);

                    Fir2Store(k,1,count) = pos_est_fire2(k,1);
                    Fir2Store(k,2,count) = pos_est_fire2(k,2);
                    Fir2Store(k,3,count) = sigma_est_fire2(k,1);

                    P_trace(k,count) = trace(P);

                elseif k == ind

                    trajectories(k,:,count) = trajectories(k,:,count-1);
                    trajectories_est(k,:,count) = trajectories_est(k,:,count-1);
                    centroids_est_stor(k,:,count) = centroids_est_stor(k,:,count-1);
                    
                    trajectories(k,3,count) = enviroment_surface(trajectories(k,1,count), ...
                                                                 trajectories(k,2,count), ...
                                                                 scenario);
                    trajectories_est(k,3,count) = 0;

                    Fir1Store(k,:,count) = [0,0,0];
                    Fir2Store(k,:,count) = [0,0,0];

                    P_trace(k,count) = 0;
                    P_trace(k+1,count) = trace(P);

                elseif k > ind
                    trajectories(k,:,count) = states(k-1,:);
                    trajectories_est(k,:,count) = states_est(k-1,:);
                    centroids_est_stor(k,:,count) = centroids_est(k-1,:);

                    Fir1Store(k,1,count) = pos_est_fire1(k-1,1);
                    Fir1Store(k,2,count) = pos_est_fire1(k-1,2);
                    Fir1Store(k,3,count) = sigma_est_fire1(k-1,1);

                    Fir2Store(k,1,count) = pos_est_fire2(k-1,1);
                    Fir2Store(k,2,count) = pos_est_fire2(k-1,2);
                    Fir2Store(k,3,count) = sigma_est_fire2(k-1,1);

                    P_trace(k+1,count) = trace(P);

                end

                if k == numUAV
                    trajectories(k+1,:,count) = states(k,:);
                    trajectories_est(k+1,:,count) = states_est(k,:);
                    centroids_est_stor(k+1,:,count) = centroids_est(k,:);

                    Fir1Store(k+1,1,count) = pos_est_fire1(k,1);
                    Fir1Store(k+1,2,count) = pos_est_fire1(k,2);
                    Fir1Store(k+1,3,count) = sigma_est_fire1(k,1);

                    Fir2Store(k+1,1,count) = pos_est_fire2(k,1);
                    Fir2Store(k+1,2,count) = pos_est_fire2(k,2);
                    Fir2Store(k+1,3,count) = sigma_est_fire2(k,1);
                end

            else
                trajectories(k,:,count) = states(k,:);
                trajectories_est(k,:,count) = states_est(k,:);
                centroids_est_stor(k,:,count) = centroids_est(k,:);

                Fir1Store(k,1,count) = pos_est_fire1(k,1);
                Fir1Store(k,2,count) = pos_est_fire1(k,2);
                Fir1Store(k,3,count) = sigma_est_fire1(k,1);

                Fir2Store(k,1,count) = pos_est_fire2(k,1);
                Fir2Store(k,2,count) = pos_est_fire2(k,2);
                Fir2Store(k,3,count) = sigma_est_fire2(k,1);
            end

        end

        % UAV fail save parameters
        if UAV_FAIL && UAV_check_fail == true
            if t == fail_time
        
                numUAV = numUAV - 1;
    
                states(ind_est,:) = [];
                objective(ind_est,:) = [];
                states_est(ind_est,:) = [];
                meas_fire1(ind_est,:) = [];
                meas_fire2(ind_est,:) = [];
                LastMeas1(:,ind_est) = [];
                LastMeas1(ind_est,:) = [];
                LastMeas2(:,ind_est) = [];
                LastMeas2(ind_est,:) = [];
                invSumLastMeas1(:,ind_est) = [];
                invSumLastMeas2(:,ind_est) = [];
                Qc1 = ones(numUAV) * 1/numUAV;
                Qc2 = ones(numUAV) * 1/numUAV;
                pos_est_fire1(ind_est,:) = [];
                pos_est_fire2(ind_est,:) = [];
                sigma_est_fire1(ind_est,:) = [];
                sigma_est_fire2(ind_est,:) = [];
    
            end
        end

        %% Plots
        % real and estimated   
        if PLOT_ITERATIVE_SIMULATION

            curr_fire1_pos = pos_fire1_mov(t); 
            %curr_fire1_sig = sigma_fire1_mov(t);
            curr_fire1_sig = sigma_fire1;
            plotSimulation_function(states, states_est, centroids_est, numUAV, dimgrid, pos_est_fire1, curr_fire1_pos(1), ...
                                    curr_fire1_pos(2), sigma_est_fire1, curr_fire1_sig, x_fire2, y_fire2, sigma_fire2, ...
                                    x_water, y_water, sigma_water, Xf, Yf, Zf, dim_UAV, 3);

        end

        fprintf('Iteration n: %d / %d\n', count, tot_iter);
        
    end

    if PLOT_TRAJECTORIES
        if UAV_FAIL
            plotUAVTrajectories_function(numUAV+1, trajectories, trajectories_est);
        else
            plotUAVTrajectories_function(numUAV, trajectories, trajectories_est);
        end
        
    end

    if PLOT_COVARIANCE_TRACE 
        if UAV_FAIL
            plotCovarianceTrace(numUAV+1,P_trace);
        else
            plotCovarianceTrace(numUAV,P_trace);
        end
    end

    if PLOT_CONSENSUS 
        if UAV_FAIL
            plotConsensus_function(1, 20, numUAV+1, posFir1StoreReal, sigmaFir1StoreReal, Fir1Store);  
            plotConsensus_function(2, 30, numUAV+1, posFir2StoreReal, sigmaFir2StoreReal, Fir2Store);  
        else
            plotConsensus_function(1, 20, numUAV, posFir1StoreReal, sigmaFir1StoreReal, Fir1Store);  
            plotConsensus_function(2, 30, numUAV, posFir2StoreReal, sigmaFir2StoreReal, Fir2Store);
        end
    end

end

if ANIMATION 

    if UAV_FAIL
        numUAV = numUAV+1;
    end

    figure_size = [300, 80, 900, 700]; % Set figure size

    %% Plot 2D voronoi simulation
    figure(50)
    set(gcf, 'Position', figure_size);
    bx = subplot(1,1,1);
    axis(bx,[0 dimgrid(1) 0 dimgrid(2)]);
    xlabel(bx,'X Coordinate');
    ylabel(bx,'Y Coordinate');
    view(bx,2);
    
    for t = 1:2:count

        figure(50)
        cla(bx);
        title(bx,['2D Voronoi Simulation - Iteration:', num2str(t),'/', num2str(tot_iter)]);
        cla;
        hold(bx,'on');

        if sigmaFir1StoreReal(1,t) > 0 % If fire is extinguished, do not plot the X
            plot(bx,posFir1StoreReal(1,1,t), posFir1StoreReal(1,2,t), 'rx', 'MarkerSize', sigmaFir1StoreReal(1,t));
        else
            % Do nothing if sigmaFir1StoreReal is <= 0
        end
        if sigmaFir2StoreReal(1,t) > 0 % If fire is extinguished, do not plot the X
            plot(bx,posFir2StoreReal(1,1,t), posFir2StoreReal(1,2,t), 'rx', 'MarkerSize', sigmaFir2StoreReal(1,t));
        else
            % Do nothing if sigmaFir2StoreReal is <= 0
        end

        plot(bx,x_water,y_water,'o','Color', 'b', 'MarkerSize', sigma_water);

        for i = 1:numUAV
            % Draw the UAV pose
            drawUAV2D(trajectories(i, 1,t), trajectories(i, 2,t), trajectories(i, 4,t), dim_UAV,'k');
            drawUAV2D(trajectories_est(i, 1,t), trajectories_est(i, 2,t), trajectories_est(i, 4,t), dim_UAV,'g');  
    
            plot(bx,centroids_est_stor(i,1,t), centroids_est_stor(i,2,t), 'x', 'Color',[0.4660, 0.6740, 0.1880]);

            if Fir1Store(i,3,t) > 0
                plot(bx,Fir1Store(i,1,t), Fir1Store(i,2,t), 'x','MarkerSize', Fir1Store(i,3,t),'Color',[0.4940, 0.1840, 0.5560]);
            end
            if Fir2Store(i,3,t) > 0
                plot(bx,Fir2Store(i,1,t), Fir2Store(i,2,t), 'x','MarkerSize', Fir2Store(i,3,t),'Color',[0.4940, 0.1840, 0.5560]);
            end
            
        end
  
        vx_es = vx_Data{t};
        vy_es = vy_Data{t};
        plot(bx,vx_es, vy_es,'Color',[0.4660, 0.6740, 0.1880]);
    
        hold(bx,'off');
        drawnow; 
        
    end   



    %% Plot 3D Simulation
    figure(60);
    set(gcf, 'Position', figure_size);
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
    alpha = 0.2;  % <-- regola questo valore

    % Mix tra texture e sfondo
    blended_texture = (alpha * texture_resized + (1 - alpha) * green_background);

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
    r = sigma_water/2 + 20 ;
    x_circle = r*cos(theta)+x_water;
    y_circle = r*sin(theta)+y_water;
    z_circle = 5 * ones(size(theta));
    hWater = plot3(ax,x_circle, y_circle, z_circle, 'b', 'LineWidth', 2);
    
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
    hFireReal1 = plot3(ax,NaN, NaN, NaN, 'xr', 'LineWidth', 2);
    hFireEst1  = plot3(ax,NaN, NaN, NaN, 'x',  'MarkerSize', 8);    
    hFireReal2 = plot3(ax,NaN, NaN, NaN, 'xr', 'LineWidth', 2);
    hFireEst2  = plot3(ax,NaN, NaN, NaN, 'x',  'MarkerSize', 8);
    
    
    for t = 1:count
        title(['3D Simulation - Iteration:', num2str(t),'/', num2str(tot_iter)]);

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
    
        % ─── update the “real” fire1 marker ────────────────────
        if sigmaFir1StoreReal(1,t) > 0
            set(hFireReal1, ...
                'XData', posFir1StoreReal(1,1,t), ...
                'YData', posFir1StoreReal(1,2,t), ...
                'ZData', enviroment_surface(...
                        posFir1StoreReal(1,1,t), ...
                        posFir1StoreReal(1,2,t), 1), ...
                'MarkerSize', sigmaFir1StoreReal(1,t));
        else
            set(hFireReal1, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        end
    
        % ─── update the “estimated” fire1 marker ───────────────
        if Fir1Store(i,3,t) > 0
            set(hFireEst1, ...
                'XData', Fir1Store(i,1,t), ...
                'YData', Fir1Store(i,2,t), ...
                'ZData', enviroment_surface(...
                        Fir1Store(i,1,t), ...
                        Fir1Store(i,2,t), 1), ...
                        'MarkerSize', Fir1Store(i,3,t),...
                        'MarkerEdgeColor', [1.0, 0.5, 0.0]);
        else 
            set(hFireEst1, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        end
        % ─── update the “real” fire2 marker ────────────────────
        if sigmaFir2StoreReal(1,t) > 0
            set(hFireReal2, ...
                'XData', posFir2StoreReal(1,1,t), ...
                'YData', posFir2StoreReal(1,2,t), ...
                'ZData', enviroment_surface(...
                        posFir2StoreReal(1,1,t), ...
                        posFir2StoreReal(1,2,t), 1), ...
                'MarkerSize', sigmaFir2StoreReal(1,t));
        
        else 
            set(hFireReal2, 'XData', NaN, 'YData', NaN, 'ZData', NaN);    
        end
        % ─── update the “estimated” fire2 marker ───────────────
        if Fir2Store(i,3,t) > 0
            set(hFireEst2, ...
                'XData', Fir2Store(i,1,t), ...
                'YData', Fir2Store(i,2,t), ...
                'ZData', enviroment_surface(...
                        Fir2Store(i,1,t), ...
                        Fir2Store(i,2,t), 1), ...
                'MarkerSize', Fir2Store(i,3,t),...
                'MarkerEdgeColor', [1.0, 0.5, 0.0]); % Orange color
        else
            set(hFireEst2, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        end

        hold(ax,'off');
        drawnow; 

        pause(0.025); % Pause to control the speed of the animation
        
    end

end

   