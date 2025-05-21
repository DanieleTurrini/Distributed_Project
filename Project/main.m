clear;
close all;
clc;

% main.m
% This script simulates a fleet of UAVs collaborating to detect and extinguish multiple dynamic fires in a 3D environment.
% Each UAV estimates fire positions and sizes using sensors and an Extended Kalman Filter, coordinates actions via consensus algorithms,
% manages water refills, and handles possible UAV failures. The simulation visualizes UAV trajectories, fire evolution, and system performance.
% Goal: Evaluate the cooperative and robust capability of the UAV fleet to extinguish moving fires in a distributed manner.

%% Simulation Parameters 

dt = 0.01;                                      % Time step
T_sim = 30;                                     % Simulation time
scenario = 1;                                   % Environment choosen
tot_iter = round((T_sim - 1)/dt + 1);           % Total number of iterations

DO_SIMULATION = true;
UAV_FAIL = false;

PLOT_ENVIRONMENT = false;
PLOT_DENSITY_FUNCTIONS = false;
PLOT_TRAJECTORIES = false;
PLOT_COVARIANCE_TRACE = true;
PLOT_CONSENSUS = false;
PLOT_EKF_ERROR = false;

PLOT_ITERATIVE_SIMULATION = false;
ANIMATION = true;

if ANIMATION == true
    DO_SIMULATION = true;
end

%% Vehicles Parameters 

vel_lin_max = 100;                  % Maximum linear velocity [m/s]
vel_lin_min = 50;                   % Minimum linear velocity [m/s]
vel_lin_z_max = 100;                % Maximum linear velocity along z [m/s]
vel_ang_max = 10;                   % Maximum angular velocity [rad/s]
dim_UAV = 4;                        % Dimension of the UAV
numUAV = 7;                         % Number of UAV
totUAV = numUAV;                    % Initial Number of UAV
Kp_z = 100;                         % Proportional gain for the linear velocity along z
Kp = 50;                            % Proportional gain for the linear velocity  
Ka = 10;                            % Proportional gain for the angular velocity
height_flight = 30;                 % Height of flight from the ground 

% Take Off
takeOff = true;
freq_takeOff = 60;                  % Time distance between each takeoff 
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
    z(i) = environment_surface(x(i),y(i),scenario) + 0.2;
end

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

UAV_check_fail = false;             % Check if the UAV is failed
fail_time = 9;                      % Time instant when one UAV fail

if fail_time > T_sim
    UAV_FAIL = false;
end

ind = 1;                            % UAV that fails
ind_est = 0;                        % Initialization of ind_est
check = ones(numUAV, 1);            % Variable that they periodically exchange 
check_treshold = 10;                % If the check of that UAV is 1 for 10 times,
                                    % it is considered failed 
check_once = true;
check_count = zeros(numUAV, 1);
communication_prob = 0.05;          % Probability of NOT communication

%% Measurement Parameters

% Measurements frequencies
meas_freq_GPS = 1; % 10 Hz
meas_freq_ultr = 4; % 25 Hz
meas_freq_gyr = 1; % 0 Hz

% Standard deviations
std_gps = 3;                                                        % Standard deviation of the GPS
std_ultrasonic = 1.5;                                               % Standard deviation of the ultrasonic sensor
std_gyro = 1;                                                     % Standard deviation of the gyroscope
R = diag([std_gps^2, std_gps^2, std_ultrasonic^2, std_gyro^2]);     % Covariance of the measurement noise

%% Kalman Filter Parameters

% Jacobian of the state model
A = @(u, theta, deltat) [1, 0, 0, -u(1) * sin(theta) * deltat;
                         0, 1, 0,  u(1) * cos(theta) * deltat;
                         0, 0, 1,                           0;
                         0, 0, 0,                           1];

% Matrix of the propagation of the process noise 
G = @(theta, deltat) [cos(theta) * deltat,      0,      0;
                      sin(theta) * deltat,      0,      0;
                                        0, deltat,      0;
                                        0,      0, deltat];

% Covariance of the process noise
std_u = [1, 1, 1];                % Uncertainty on the linear velocity in x-y,
                                    % linear velocity in z and angular velocity

Q = diag(std_u.^2);                 % Covariance matrix of model and control uncertanty

% Initial state (x,y,z,theta)
states_est = (states' + [std_gps * randn(2, numUAV); zeros(1, numUAV); std_gyro * randn(1, numUAV)])';


% Observation matrix (H,h)
h = @(s,theta_old,dt,err) [s(:,1)+err(:,1),  s(:,2)+err(:,2),  s(:,3)+err(:,3),  (s(:,4)-theta_old)/dt+err(:,4)]; % We measure the position and the angle of the UAV 

% H = dh/dx|err=0
H = [1, 0, 0,    0;
     0, 1, 0,    0;
     0, 0, 1,    0;
     0, 0, 0, 1/dt];        % (we measure theta with a gyroscope, 
                            %  it measure the angular velocity)

% Covariance matrix of the initial estimate
P = zeros(4,4,numUAV);
for k = 1:numUAV
    P(:,:,k) = eye(4) * 5;   % Consideriamo un'incertezza iniziale per ogni UAV
end

%% Map Parameters

dimgrid = [500 500 500];                    % Define the dimensions of the Map

[Xf, Yf, Zf] = plot_environment_surface(PLOT_ENVIRONMENT);

%% Fires Parameters

drop_dist = 0.6;         % Percentage of the distance from the center of 
                         % the fire that has to be reached until drop the water 

% Fires Positions
x_fire1 = 300;
y_fire1 = 400;
x_fire2 = 450;
y_fire2 = 50;

pos_fire1_start = [x_fire1 , y_fire1];
pos_fire2_start = [x_fire2 , y_fire2];

% Simulated moving fire 1
pos_fire1_mov = @(t) [x_fire1 - 5 * (t - 1) , y_fire1 - 2 * (t - 1)]; % t start from 1

sigma_fire1 = 50;                           % Standard deviation of the first fire
                                            % (corresponding to the extention of the fire)

% Simulated moving fire 2
pos_fire2_mov = @(t) [x_fire2 - 2.5 * (t - 1) , y_fire2 + 1 * (t - 1)]; % t start from 1

sigma_fire2 = 25;                           % Standard deviation of the second fire
                                            % (corresponding to the extention of the fire)

inc_threshold1 = sigma_fire1 * drop_dist;   % Distance that has to be reach from the fire 1 
inc_threshold2 = sigma_fire2 * drop_dist;   % Distance that has to be reach from the fire 2

trashold_sigma_fire = 10;                   % If the sigma is less than a trashold, we set it to 0

pos_est_fire1 = zeros(numUAV,2);
sigma_est_fire1 = zeros(numUAV,1);
pos_est_fire2 = zeros(numUAV,2);
sigma_est_fire2 = zeros(numUAV,1);

for i = 1:numUAV

    % --- Fire 1 ---
    pos_est_fire1(i,:) = pos_fire1_mov(1);                 % Initialize the estimated positions of fire 1
    sigma_est_fire1(i,1) = sigma_fire1;                    % Initialize the estimated extension of fire 1

    % --- Fire 2 ---
    pos_est_fire2(i,:) = pos_fire2_mov(1);                 % Initialize the estimated positions of fire 2
    sigma_est_fire2(i,1) = sigma_fire2;                    % Initialize the estimated extension of fire 2

end

% Decreasing factor of the fire
deacreasingFire_factor = 4;                 % Decreasing factor of the fire extension
                                            % (we assume that the fire decrease every time the UAV drop the water)
                        
%% Water Parameters

% Water Positions
x_water = 50;
y_water = 50;

pos_water = [x_water, y_water];

sigma_water = 40;
wat_threshold = 30;                         % Distance that has to be reach from the water source to refill

%%  Density Functions for the fires and the water

[G_fire,G_water] = objective_density_functions(dimgrid, pos_fire1_mov, pos_fire2_mov, pos_water, sigma_fire1, ...
                                               sigma_fire2, sigma_water, 0, initialUAV_pos, PLOT_DENSITY_FUNCTIONS);


%% Consensus Parameters

sensor_range = 70;                          % Infrared measurement distance [m]

meas_fire1 = zeros(numUAV,1);               % Variable used to perform a fire measurement just once
meas_fire2 = zeros(numUAV,1);               % Variable used to perform a fire measurement just once

% Each UAV has an estimate of the measurement time of the other UAVs (we add some uncertanty)
LastMeas1 = ones(numUAV,numUAV) + 6 * rand(numUAV,numUAV) - 3;           % At the beginning no one UAV has done a measurement
LastMeas2 = ones(numUAV,numUAV) + 6 * rand(numUAV,numUAV) - 3;           % At the beginning no one UAV has done a measurement

invSumLastMeas1 = ones(1,numUAV);
invSumLastMeas2 = ones(1,numUAV);

Qc1 = ones(numUAV) * 1/numUAV;               % Initialization of matrix Q for fire 1
Qc2 = ones(numUAV) * 1/numUAV;               % Initialization of matrix Q for fire 2
 

%% Save Matrices Declaration

% Real Trajectories
trajectories = zeros(numUAV, 4, tot_iter);

% Estimated Trajectories
trajectories_est = zeros(numUAV, 4, tot_iter);

% Estimation error 
est_error = zeros(numUAV, 4, tot_iter);

% Save all the traces of P
P_trace = zeros(numUAV,tot_iter);

% Centroids Trajectories
centroids_est_stor = zeros(numUAV, 2, tot_iter);


Fir1Store = zeros(numUAV, 3, tot_iter);

% Estimated Trajectory of X coordinate of fire 1
Fir1Store(:,1,1) = pos_est_fire1(:,1);
% Estimated Trajectory of Y coordinate of fire 1
Fir1Store(:,2,1) = pos_est_fire1(:,2);
% Behavior of the extension of fire 1
Fir1Store(:,3,1) = sigma_est_fire1(:,1);


Fir2Store = zeros(numUAV, 3, tot_iter);

% Estimated Trajectory of X coordinate of fire 2
Fir2Store(:,1,1) = pos_est_fire2(:,1);
% Estimated Trajectory of Y coordinate of fire 2
Fir2Store(:,2,1) = pos_est_fire2(:,2);
% Behavior of the extension of fire 2
Fir2Store(:,3,1) = sigma_est_fire2(:,1);

% Real Path of Fire 1
posFir1StoreReal = zeros(1, 2, tot_iter);
sigmaFir1StoreReal = zeros(1, tot_iter);

% Real Path of Fire 2
posFir2StoreReal = zeros(1, 2, tot_iter);
sigmaFir2StoreReal = zeros(1, tot_iter);

% Initialization of the distances between fires and UAVs
dist_inc1 = zeros(numUAV,1); 
dist_real_inc1 = zeros(numUAV,1);
dist_inc2 = zeros(numUAV,1);
dist_real_inc2 = zeros(numUAV,1);

% Voronoi Edges
vx_Data = cell(1, tot_iter);    % Cells to store vx data for each iteration
vy_Data = cell(1, tot_iter);    % Cells to store vy data for each iteration

% Create a grid of points
[x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));

% Drops time distance
drop_times_diff = zeros(1,1);
drop_times = zeros(1,1);

% Measurementof the state
measurements = zeros(numUAV, 4, tot_iter); % Initialize the measurements matrix 

% Number of drops 
drops_f1 = 0;
drops_f2 = 0;


%% Simulation
if DO_SIMULATION
    
    count = 0;
    for t = 1:dt:T_sim
        
        % Updates 
        count = count + 1;
        LastMeas1 = LastMeas1 + 1;
        LastMeas2 = LastMeas2 + 1;

        %% Check commumication

        % See if communication is present
        for k = 1:totUAV
            if rand(1) < communication_prob && count > 1

                check(k) = 0;       % NO communication
                fprintf('No communication for UAV %d\n', k);

            else

                check(k) = 1;       % YES communication

            end

            if UAV_FAIL && t >= fail_time + dt

                check(ind) = 0;     % Impose no communication for the UAV crashed

            end

            if check_count(k) >= check_treshold      % If for some steps i did't recived any message, consider the UAV crashed

                disp('Found a UAV crash');
                UAV_check_fail = true;
                ind_est = k;
                
            end

            if check(k) == 0    % NO communication

                check_count(k) = check_count(k) + 1;

                % Set the previous position and fire estimation
                if UAV_check_fail == false && k ~= ind_est

                    states_est(k,:) = trajectories_est(k,:,count-1);
                    pos_est_fire1(k,:) = Fir1Store(k,1:2,count-1);
                    pos_est_fire2(k,:) = Fir2Store(k,1:2,count-1);
                    sigma_est_fire1(k,1) = Fir1Store(k,3,count-1);
                    sigma_est_fire2(k,1) = Fir2Store(k,3,count-1);

                end
                
            else

                check_count(k) = 0;

            end
    
        end 

        % Real fire position and extension

        % Fire 1
        posFir1StoreReal(1,:,count) = pos_fire1_mov(t);
        sigmaFir1StoreReal(1,count) = sigma_fire1;

        % Fire 2
        posFir2StoreReal(1,:,count) = pos_fire2_mov(t);
        sigmaFir2StoreReal(1,count) = sigma_fire2;

        dist_inc1 = zeros(numUAV,1); 
        dist_real_inc1 = zeros(numUAV,1);
        
        dist_inc2 = zeros(numUAV,1);
        dist_real_inc2 = zeros(numUAV,1);

        dist_wat  = pdist2(pos_water, states_est(:,1:2));               % Distance to the water source

        
        dist_real_inc1(:,1) = pdist2(pos_fire1_mov(t),states(:,1:2));   % Here we use the real posititon since we are 
                                                                        % considering if the sensor are able to detect the fire

        dist_real_inc2(:,1) = pdist2(pos_fire2_mov(t),states(:,1:2));   % Here we use the real posititon since we are 
                                                                        % considering if the sensor are able to detect the fire

        % Verify if the wanted distance from the target is reached
        for i = 1:numUAV
            
            dist_inc1(i) = pdist2(pos_est_fire1(i,:), states_est(i,1:2));   % Distance to the first fire
            dist_inc2(i) = pdist2(pos_est_fire2(i,:), states_est(i,1:2));   % Distance to the second fire
            inc_threshold1(i) = sigma_est_fire1(i,1) * drop_dist;           % Distance that has to be reach from the fire 2
            inc_threshold2(i) = sigma_est_fire2(i,1) * drop_dist;           % Distance that has to be reach from the fire 2

            % --- Fire 1 ---
            if dist_real_inc1(i) <= sensor_range && objective(i) == 1 && meas_fire1(i) ~= 1 

                % Meaurement
                pos_est_fire1(i,:) = pos_fire1_mov(t) + 10 * rand - 5;      % high uncertanty since the measurement 
                                                                            % is done by camera and infrared and the
                                                                            % fire could have different complex shapes 

                sigma_est_fire1(i,1) = sigma_fire1 + 4 * rand - 2;          

                LastMeas1(i,:) = 1 + (2 * rand(1,numUAV) - 1);  % Set to 1 the LastMeas with some uncertanty becouse the
                                                              % messages could arrive to the other UAVs with some delay 
                invLastMeas1 = 1 ./ LastMeas1;

                for j = 1:numUAV

                    invSumLastMeas1(j) = sum(invLastMeas1(:,j));

                    for k = 1:numUAV

                        Qc1(j,k) = (invLastMeas1(k,j)) / (invSumLastMeas1(j));  % Update the matrix Q

                    end

                end

                meas_fire1(i) = 1;      % The measurement has been done

            end

            % --- Fire 2 ---
            if dist_real_inc2(i) <= sensor_range && objective(i) == 1 && meas_fire2(i) ~= 1

                % Meaurement
                pos_est_fire2(i,:) = pos_fire2_mov(t) + 10 * rand(1,1) - 5;       % high uncertanty since the measurement 
                                                                                  % is done by camera and infrared and the
                                                                                  % fire could have different complex shapes 

                sigma_est_fire2(i,1) = sigma_fire2 + 4 * rand(1,1) - 2;           

                LastMeas2(i,:) = 1 + 2 * rand(1,numUAV) - 1;  % Set to 1 the LastMeas with some uncertanty becouse the
                                                          % messages could arrive to the other UAVs with some delay 
                invLastMeas2 = 1 ./ LastMeas2;

                for j = 1:numUAV

                    invSumLastMeas2(j) = sum(invLastMeas2(:,j));

                    for k = 1:numUAV

                        Qc2(j,k) = (invLastMeas2(k,j)) / ( invSumLastMeas2(j) );  % Update the matrix Q

                    end

                end

                meas_fire2(i) = 1;

            end
            
            % If the UAV is close to a fire and its objective is 1 (heading to fire)
            if dist_inc1(i) <= inc_threshold1(i) && objective(i) == 1

                sigma_fire1 = sigma_fire1 - deacreasingFire_factor;     % Reduce the extension of the first fire 

                if sigma_fire1 <= 0

                    sigma_fire1 = 0;

                end

                objective(i) = 2; % Change objective to 2 (heading to refill water)
                meas_fire1(i) = 0;

                drop_times = [drop_times,count];                        % Save the drop's instant
                dt_drop = drop_times(end) - drop_times(end-1);          % Compute the time difference with the previous drop
                drop_times_diff = [drop_times_diff, dt_drop];
                drops_f1 = drops_f1 + 1;

            elseif dist_inc2(i) <= inc_threshold2(i) && objective(i) == 1
                
                sigma_fire2 = sigma_fire2 - deacreasingFire_factor;     % Reduce the extension of the first fire

                if sigma_fire2 <= 0

                    sigma_fire2 = 0;

                end

                objective(i) = 2; % Change objective to 2 (heading to refill water)
                meas_fire1(i) = 0;

                drop_times = [drop_times,count];                        % Save the drop's instant
                dt_drop = drop_times(end) - drop_times(end-1);          % Compute the time difference with the previous drop
                drop_times_diff = [drop_times_diff, dt_drop];
                drops_f2 = drops_f2 + 1;

            end

            % If the UAV is close to the water source and its objective is 2 (heading to refill)
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

        %% Consensus algorithm

        % We use the same matrix Q for both the coordinates and the extension

        % --- Fire 1 ---
        pos_est_fire1(:,1) = Qc1 * pos_est_fire1(:,1); % pos_est_fire(k+1) = Qc^k *  pos_est_fire(1)
        pos_est_fire1(:,2) = Qc1 * pos_est_fire1(:,2);
        sigma_est_fire1(:,1) = Qc1 * sigma_est_fire1(:,1);

        % --- Fire 2 ---
        pos_est_fire2(:,1) = Qc2 * pos_est_fire2(:,1);
        pos_est_fire2(:,2) = Qc2 * pos_est_fire2(:,2);
        sigma_est_fire2(:,1) = Qc2 * sigma_est_fire2(:,1);


        % If the sigma is under a certian value means that the fire 1 is off
        if sigma_est_fire1(:,1) < trashold_sigma_fire

            sigma_est_fire1(:,1) = 0;

        end

        % If the sigma is under a certian value means that the fire 2 is off
        if sigma_est_fire2(:,1) < trashold_sigma_fire

            sigma_est_fire2(:,1) = 0;

        end


        % Compute Voronoi tessellation and velocities
        [areas, centroids_est, control_est] = voronoi_function_FW(numUAV, dimgrid, states_est, Kp_z, Kp, Ka, ...
                                                                  pos_est_fire1, pos_est_fire2, sigma_est_fire1, sigma_est_fire2, ...
                                                                  G_water, height_flight, scenario, objective, initialUAV_pos);
                                                                  
        
        % Impose a Boundaries on velocity
        % The linear straight velocty has also a minimum velocity since we are considering Fixed wing UAV 
        control_est(:,1) = sign(control_est(:,1)) .* max(min(abs(control_est(:,1)), vel_lin_max), vel_lin_min); % Linear velocity 
        control_est(:,2) = sign(control_est(:,2)) .* min(abs(control_est(:,2)), vel_lin_z_max); % Limit linear velocity along z
        control_est(:,3) = sign(control_est(:,3)) .* min(abs(control_est(:,3)), vel_ang_max); % Limit angular velocity

        % If the UAV crashed but other UAVs don't know yet
        if UAV_FAIL && t >= fail_time + dt && UAV_check_fail == false

            control_est(ind,:) = [0,0,0];

        end 


        %% Landing control

        for i = 1:numUAV

            dist_to_initial = norm(states(i,1:2) - initialUAV_pos(i,1:2));

            if dist_to_initial < 100 && objective(i) == 3

                % If the UAV is close to the initial position, set the vertical speed
                control_est(i,2) = sign( 0.2 - states(i,3)) * min(vel_lin_z_max, Kp_z/100 * abs( 0.5 - states(i,3))); % flight_surface(initialUAV_pos(i,1),initialUAV_pos(i,1),0,1) +

                if dist_to_initial < 8

                    control_est(i,1:2) = 0;

                end

            end

        end 



        %% TakeOff 

        if takeOff && n ~= numUAV + 1

            for s = n:numUAV

                control_est(s,:) = [0,0,0]; % Keep all the UAV still until the departure

            end

            if mod(count, freq_takeOff) == 0

                n = n + 1;

            end

        end

        %% Refill control

        for k = 1:numUAV

            if count_refill(k) ~= 0

                control_est(k,1) = vel_lin_min ;            % Set the velocity to minimum during the refill
                control_est(k,3) = 0;                       % The UAV will go straight during the refill
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

        %% Measure
        measure = h(states, states_est(:,4), dt, [  std_gps * randn(numUAV,1 ), ...
                                            std_gps * randn(numUAV,1), ...
                                            std_ultrasonic * randn(numUAV,1), ...
                                            std_gyro * randn(numUAV,1)]);

        %{
         measure = (H * states' + [std_gps * randn(2, numUAV); ...
                                  std_ultrasonic * randn(1, numUAV); ...
                                  std_gyro * randn(1, numUAV)])'; 
        %}


        %% Extended Kalman Filter
        for k = 1:numUAV

            [states_est(k,:), P(:,:,k)] = ExtendedKalmanFilter_function(states_est(k,:), measure(k,:), control_est(k,:), ...
                A, G, fun, Q, h, H, R, P(:,:,k), count, ...
                meas_freq_GPS, meas_freq_ultr, meas_freq_gyr, dt);

            P_trace(k,count) = trace(P(:,:,k));
        end


        %{
                for k = 1:numUAV

                    [states_est(k,:), P] = ExtendedKalmanFilter_function(states_est(k,:), measure(k,:), control_est(k,:), ...
                                                                        A, G, fun, Q, h, H, R, P, count, ...
                                                                        meas_freq_GPS, meas_freq_ultr, meas_freq_gyr, dt);

                    P_trace(k,count) = trace(P);

                end 
        %}


        % Save Voronoi edges
        [vx, vy] = voronoi(states_est(:,1), states_est(:,2));
        vx_Data{count} = vx;
        vy_Data{count} = vy;


        %% UAV fail 

        for k = 1:totUAV

            % UAV fail save parameters
            if UAV_FAIL && UAV_check_fail == true && check_once
            
                numUAV = numUAV - 1;
                
                % Remove the UAV crashed
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
                check(k) = [];
    
                check_once = false;
        
            end

            % Storing data properly during UAV fail
            if UAV_FAIL && t >= fail_time + dt && UAV_check_fail
                if k < ind
                    measurements(k, :, count) = measure(k,:); 
                    est_error(k,:,count) = abs(states_est(k,:) - states(k,:));
                    trajectories(k,:,count) = states(k,:);
                    trajectories_est(k,:,count) = states_est(k,:);
                    centroids_est_stor(k,:,count) = centroids_est(k,:);

                    Fir1Store(k,1,count) = pos_est_fire1(k,1);
                    Fir1Store(k,2,count) = pos_est_fire1(k,2);
                    Fir1Store(k,3,count) = sigma_est_fire1(k,1);

                    Fir2Store(k,1,count) = pos_est_fire2(k,1);
                    Fir2Store(k,2,count) = pos_est_fire2(k,2);
                    Fir2Store(k,3,count) = sigma_est_fire2(k,1);

                    P_trace(k,count) = trace(P(:,:,k));
                    

                elseif k == ind
                    
                    measurements(k, :, count) = [0, 0, 0, 0]; 
                    est_error(k,:,count) = 0;
                    trajectories(k,:,count) = trajectories(k,:,count-1);
                    trajectories_est(k,:,count) = trajectories_est(k,:,count-1);
                    centroids_est_stor(k,:,count) = centroids_est_stor(k,:,count-1);
                    
                    trajectories(k,3,count) = environment_surface(trajectories(k,1,count), ...
                                                                 trajectories(k,2,count), ...
                                                                 scenario);
                    trajectories_est(k,3,count) = 0;

                    Fir1Store(k,:,count) = [0,0,0];
                    Fir2Store(k,:,count) = [0,0,0];

                    P_trace(k,count) = 0;
                    P_trace(k+1,count) = trace(P(:,:,k));
                elseif k > ind

                    measurements(k, :, count) = measure(k-1,:); 
                    est_error(k,:,count) = abs(states_est(k-1,:) - states(k-1,:));
                    trajectories(k,:,count) = states(k-1,:);
                    trajectories_est(k,:,count) = states_est(k-1,:);
                    centroids_est_stor(k,:,count) = centroids_est(k-1,:);

                    Fir1Store(k,1,count) = pos_est_fire1(k-1,1);
                    Fir1Store(k,2,count) = pos_est_fire1(k-1,2);
                    Fir1Store(k,3,count) = sigma_est_fire1(k-1,1);

                    Fir2Store(k,1,count) = pos_est_fire2(k-1,1);
                    Fir2Store(k,2,count) = pos_est_fire2(k-1,2);
                    Fir2Store(k,3,count) = sigma_est_fire2(k-1,1);

                    P_trace(k+1,count) = trace(P(:,:,k));

                end

            else

                measurements(k, :, count) = measure(k,:); 
                est_error(k,:,count) = abs(states_est(k,:) - states(k,:));
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

        

        %% Plots
        % real and estimated   
        if PLOT_ITERATIVE_SIMULATION

            curr_fire1_pos = pos_fire1_mov(t); 
            curr_fire1_sig = sigma_fire1;
            plotSimulation_function(states, states_est, centroids_est, numUAV, dimgrid, pos_est_fire1, curr_fire1_pos(1), ...
                                    curr_fire1_pos(2), sigma_est_fire1, curr_fire1_sig, x_fire2, y_fire2, sigma_fire2, ...
                                    x_water, y_water, sigma_water, Xf, Yf, Zf, dim_UAV, 3);

        end

        fprintf('Iteration n: %d / %d\n', count, tot_iter);
        
    end

    % SIMULATION EVALUATION 
    
    disp(sprintf('Mean drop time distance: %f ', mean(drop_times_diff(3:end))));
    totalDrops = numel(drop_times_diff) - 1;
    disp(sprintf('Total Number of drops: %d', totalDrops));

    disp(sprintf('Number of drops on the Fire 1: %d', drops_f1));
    disp(sprintf('Number of drops on the Fire 2: %d', drops_f2));

    if PLOT_TRAJECTORIES

        if UAV_FAIL

            plotUAVTrajectories_function(numUAV+1, trajectories, trajectories_est,dimgrid(1:2),measurements);

        else

            plotUAVTrajectories_function(numUAV, trajectories, trajectories_est,dimgrid(1:2),measurements);

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

    if PLOT_EKF_ERROR

        if UAV_FAIL

            plotEKFerror(numUAV+1, est_error);
                
        else

            plotEKFerror(numUAV, est_error);
            
        end

    end

end

if ANIMATION 

    if UAV_FAIL

        numUAV = numUAV + 1;
          
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

        if sigmaFir1StoreReal(1,t) > 10 % If fire is extinguished, do not plot the X

            plot(bx,posFir1StoreReal(1,1,t), posFir1StoreReal(1,2,t), 'rx', 'MarkerSize', sigmaFir1StoreReal(1,t));

        end

        if sigmaFir2StoreReal(1,t) > 10 % If fire is extinguished, do not plot the X

            plot(bx,posFir2StoreReal(1,1,t), posFir2StoreReal(1,2,t), 'rx', 'MarkerSize', sigmaFir2StoreReal(1,t));

        end

        plot(bx,x_water,y_water,'o','Color', 'b', 'MarkerSize', 90);

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
    
    % Loading texture
    texture = imread('Mountain.jpg');  % Must be RGB
    texture_resized = imresize(texture, [size(Xf,1), size(Xf,2)]);
    texture_resized = double(texture_resized) / 255;  % Normalize between 0 and 1

    % Increase brightness
    brightness_factor = 1.5;
    texture_resized = min(texture_resized * brightness_factor, 1);

    % Create a uniform green background
    green_background = repmat(reshape([0.4660 0.6740 0.1880], 1, 1, 3), size(texture_resized, 1), size(texture_resized, 2));

    % Transparency factor (0 = only green background, 1 = only texture)
    alpha = 0.2;  % <-- adjust this value

    % Blend texture and background
    blended_texture = (alpha * texture_resized + (1 - alpha) * green_background);

    % ─── static terrain ───────────────────────────────
    surf(ax, Xf, Yf, Zf, 'CData', flip(blended_texture, 1), ...
        'FaceColor', 'texturemap', ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'none');

    lighting none;

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
            z0  = environment_surface(x, y, 1);
    
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
                'ZData', environment_surface(...
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
                'ZData', environment_surface(...
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
                'ZData', environment_surface(...
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
                'ZData', environment_surface(...
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

   