clear all;
close all;
clc;

%% Simulation Parameters 

dt = 0.01;
T_sim = 3;
scenario = 1;

DO_SIMULATION = true;
PLOT_DENSITY_FUNCTIONS = true;
PLOT_TRAJECTORIES = true;
PLOT_ITERATIVE_SIMULATION = true;
PLOT_CONSENSUS = true;

%% Vehicles Parameters 

vel_lin_max = 600;  % Maximum linear velocity
vel_lin_min = 30;   % Minimum linear velocity
vel_lin_z_max = 50; % Maximum linear velocity along z
vel_ang_max = 30;  % Maximum angular velocity
dim_UAV = 3;  % Dimension of the UAV
numUAV = 5;   % Number of UAV
Kp_z = 10;  % Proportional gain for the linear velocity along z
Kp = 50;   % Proportional gain for the linear velocity  
Ka = 15;   % Proportional gain for the angular velocity 
Ke = 10;   % Additional gain for the angular velocity 
height_flight = 30;   % Height of flight from the ground 

% Generate random starting positions for each point
x = rand(numUAV, 1) * 100;   % Random x coordinates
y = 100 + rand(numUAV, 1) * 100;  % Random y coordinates
z = zeros(numUAV,1);    % Start from ground
theta = zeros(numUAV,1);

states = [x, y, z, theta];

objective = ones(numUAV,1);     % -> objective = 1 : the UAV is filled with
                                % water and is going to put out the fire
                                % -> objective = 2 : the UAV is empty and is
                                % going to refill
                                % (we assume every one empty at the beginning)

objective_est = ones(numUAV,1); % the objective of the estimated positions

% Dynamics

fun = @(state, u, deltat) [state(1) + u(1) * cos(state(4)) * deltat, ...
                           state(2) + u(1) * sin(state(4)) * deltat, ...
                           state(3) + u(2) * deltat, ...
                           state(4) + u(3) * deltat];

%% Kalman Filter Parameters

% Jacobian of the state model
% !!! WE ASSUME THE CONTROL AS INDIPENDENT FROM THE STATE -> NOT TRUE !!!
A = @(u, theta, deltat) [1, 0,    0, -u(1) * sin(theta) * deltat;
                         0, 1,    0,  u(2) * cos(theta) * deltat;
                         0, 0,    1,                           0;
                         0, 0,    0,                           1];

% Matrix of the propagation of the process noise for (x,y,z,theta) 4x4
% We considered the niose as white and related to the uncertanty in the
% control (is our uncertanty in the model)
G = @(theta, deltat) [cos(theta) * deltat,      0,      0;
                      sin(theta) * deltat,      0,      0;
                                        0, deltat,      0;
                                        0,      0, deltat];

% Covariance of the process noise
std_u = [5, 5, 5]; % Uncertainty on the velocity (tang , z) and angular velocity
Q = diag(std_u.^2);

% Measurement Parameters

% Measurements frequency [cs]
meas_freq_GPS = 10;
meas_freq_ultr = 2;
meas_freq_gyr = 5;

std_gps = 5; % Standard deviation of the GPS
std_ultrasonic = 2; % Standard deviation of the ultrasonic sensor
std_gyro = 0.5; % Standard deviation of the gyroscope
R = diag([std_gps^2, std_gps^2, std_ultrasonic^2, std_gyro^2]); % Covariance of the measurement noise


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
P = eye(4) * 20; % We consider some starting uncertanty

% Map Parameters
dimgrid = [500 500 500];   % Define the dimensions of the grid

%% Fires Parameters

% Fires Positions
x_fire1 = 400;
y_fire1 = 400;
x_fire2 = 450;
y_fire2 = 50;

pos_fire1 = [x_fire1 , y_fire1];
pos_fire1_mov = @(s) [x_fire1 - 0.2 * s , y_fire1];
pos_fire2 = [x_fire2, y_fire2];

sigma_fire1 = 15;   % Standard deviation of the first fire
                    % (correspond to the extention of the fire)

sigma_fire2 = 15;   % Standard deviation of the second fire
                    % (correspond to the extention of the fire)

inc_threshold1 = sigma_fire1;  % Distance that has to be reach from the fire 1 
inc_threshold2 = sigma_fire2;  % Distance that has to be reach from the fire 2

pos_est_fire1 = zeros(numUAV,2);

for i = 1:numUAV
    pos_est_fire1(i,:) = pos_fire1_mov(0); % Initialize the positions of the 
end

%% Water Parameters

% Water Positions
x_water = 50;
y_water = 50;
pos_water = [x_water, y_water];

sigma_water = 60;
wat_threshold = sigma_water;   % Distance that has to be reach from the water source to refill

% Density Functions for the fires and the water
[G_fire,G_water] = objective_density_functions(dimgrid, pos_fire1_mov, pos_fire2, pos_water, sigma_fire1, sigma_fire2, sigma_water,0, PLOT_DENSITY_FUNCTIONS);

plot_flight_surface();

trajectories = zeros(numUAV, 4, (T_sim-1)/dt);
trajectories_est = zeros(numUAV, 4, (T_sim-1)/dt);

% Save all the traces of P
P_trace = zeros(numUAV,(T_sim-1)/dt);

%% Consensus Parameters

LastMeas = ones(numUAV,1) * 1000; 
Qc = ones(numUAV) * 1/numUAV;
posFir1StoreX = zeros(numUAV,1,(T_sim-1)/dt);
posFir1StoreX(:,1,1) = pos_est_fire1(:,1);

%% Simulation
if DO_SIMULATION
    
    count = 0;
    for t = 1:dt:T_sim

        count = count + 1;
        
        % % Control algorithm - IDEAL
        % % (control computed considering the real positions)
        % [control, objective, centroids] = control_computation(numUAV, dimgrid, states, objective, ...
        %     pos_fire1, pos_fire2, pos_water, inc_threshold1, inc_threshold2, wat_threshold, ...
        %     Kp_z, Kp, Ka, Ke, G_fire, G_water, scenario, vel_lin_max, vel_lin_min, vel_lin_z_max, vel_ang_max);

        % Model Simulation - ESTIMATED
        % [control_est, objective_est, centroids_est] = control_computation(numUAV, dimgrid, states_est, objective_est, ...
        %     LastMeas, count, pos_fire1, pos_est_fire1, pos_fire1_mov, pos_fire2, pos_water, inc_threshold1, inc_threshold2, wat_threshold, ...
        %     Kp_z, Kp, Ka, Ke, G_fire, G_water, scenario, vel_lin_max, vel_lin_min, vel_lin_z_max, vel_ang_max, t);
        

        % Compute the distances from fires and water source for each drone
    
        % dist_inc1 = pdist2(pos_fire1, states(:,1:2)); % Distance to the first fire
        dist_inc1 = zeros(numUAV,2);
    
        dist_inc2 = pdist2(pos_fire2, states(:,1:2)); % Distance to the second fire
        dist_wat  = pdist2(pos_water, states(:,1:2)); % Distance to the water source
        
        % Verify if the wanted distance from the target is reached
        for i = 1:numUAV
            dist_inc1(i) = pdist2(pos_est_fire1(i,:), states(i,1:2));
    
            % If the drone is close to a fire and its objective is 1 (heading to fire)
            if dist_inc1(i) <= inc_threshold1  && objective(i) == 1

                objective(i) = 2; % Change objective to 2 (heading to refill water)
           
                pos_est_fire1(i,:) = pos_fire1_mov(t);
                LastMeas(i) = count;
                
                % Definition of Q
                for j = 1:numUAV
                    if j ~= i
                        Qc(:,j) = 1/LastMeas(j) + 0.2 * rand(1,numUAV);

                    end
                end
                
                for s = 1:numUAV
                    Qc(s,i) = 1 - (sum(Qc(s,:))-Qc(s,i));
                end


            elseif dist_inc2(i) <= inc_threshold2 && objective(i) == 1
                objective(i) = 2; % Change objective to 2 (heading to refill water)
            end

            % If the drone is close to the water source and its objective is 2 (heading to refill)
            if dist_wat(i) <= wat_threshold && objective(i) == 2
                objective(i) = 1; % Change objective to 1 (heading to fire)
            end
        end


        % Consensus algorithm
        disp(Qc);
        pos_est_fire1(:,1) = Qc * pos_est_fire1(:,1);
        posFir1StoreX(:,1,count+1) = pos_est_fire1(:,1);

    
        % Compute Voronoi tessellation and velocities
        [areas, centroids_est, control_est] = voronoi_function_FW(numUAV, dimgrid, states, Kp_z, Kp, Ka, Ke, G_fire, G_water, scenario, objective);
    
        % Impose a maximum velocity
        % The linear straight velocty has also a minimum velocity since we are considering Fixed wing UAV 
        control_est(:,1) = sign(control_est(:,1)) .* max(min(abs(control_est(:,1)), vel_lin_max), vel_lin_min); % Linear velocity 
        control_est(:,2) = sign(control_est(:,2)) .* min(abs(control_est(:,2)), vel_lin_z_max); % Limit linear velocity along z
        control_est(:,3) = sign(control_est(:,3)) .* min(abs(control_est(:,3)), vel_ang_max); % Limit angular velocity


        % Model Simulation - REAL 
        % we use to control the real model the velocities computed using
        % the estimated states (as it will be in real applications)

        for k = 1:numUAV
            states(k,:) = fun(states(k,:), control_est(k,:), dt);
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
        end
        
        for k = 1:numUAV
            trajectories_est(k,:,count) = states_est(k,:);
        end

%% Plots
        % real and estimated   
        if PLOT_ITERATIVE_SIMULATION

            plotSimulation_function(states, states_est, centroids_est, numUAV, dimgrid, x_fire1, y_fire1, sigma_fire1, x_fire2, y_fire2, sigma_fire2, x_water, y_water, sigma_water,dim_UAV, 3);

        end


    end

    if PLOT_TRAJECTORIES
        
        for i = 1:numUAV

            figure(4 + i)
            subplot(4,1,1);
            plot(squeeze(trajectories(i,1,:)),'b');
            hold on;
            plot(squeeze(trajectories_est(i,1,:)),'r--');
            hold off;
            xlabel('Time');
            ylabel('X');
            title(sprintf('X-Dimension of UAV %d',i));
            legend('Real Trajectory','Estimated Trajectory');

            subplot(4,1,2);
            plot(squeeze(trajectories(i,2,:)),'b');
            hold on;
            plot(squeeze(trajectories_est(i,2,:)),'r--');
            hold off;
            xlabel('Time');
            ylabel('Y');
            title(sprintf('Y-Dimension of UAV %d',i));
            legend('Real Trajectory','Estimated Trajectory');

            subplot(4,1,3);
            plot(squeeze(trajectories(i,3,:)),'b');
            hold on;
            plot(squeeze(trajectories_est(i,3,:)),'r--');
            hold off;
            xlabel('Time');
            ylabel('Z');
            title(sprintf('Z-Dimension of UAV %d',i));
            legend('Real Trajectory','Estimated Trajectory');

            subplot(4,1,4);
            plot(squeeze(trajectories(i,4,:)),'b');
            hold on;
            plot(squeeze(trajectories_est(i,4,:)),'r--');
            hold off;
            xlabel('Time');
            ylabel('Theta');
            title(sprintf('Theta-Dimension of UAV %d',i));
            legend('Real Trajectory','Estimated Trajectory');

            figure(5+numUAV)
            subplot(numUAV,1,i);
            plot(squeeze(P_trace(i,:)));
            xlabel('Time');
            ylabel('Covariance');
            title(sprintf('Trace of Covariance matrix of UAV %d',i));
          
        end


    end

    if PLOT_CONSENSUS 

        figure(20)
        hold on;

        for k = 1:numUAV

            plot(squeeze(posFir1StoreX(k,1,:)));

        end
        hold off;
    end

end