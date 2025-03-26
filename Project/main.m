clear all;
close all;
clc;

%% Parameters

DO_SIMULATION = true;
PLOT_DENSITY_FUNCTIONS = true;

% Vehicles Parameters 
vel_lin_max = 100; 
vel_ang_max = 20; 
dimension = 3;  % Dimension of the UAV
numUAV = 5;
Kp = 50;   % Proportional gain for the linear velocity  
Ka = 15;   % Proportional gain for the angular velocity 
Ke = 10;   % Additional gain for the angular velocity 

% Generate random starting positions for each point
x = rand(numUAV, 1) * 100;   % Random x coordinates
y = 100 + rand(numUAV, 1) * 100;  % Random y coordinates
z = zeros(numUAV,1);
theta = zeros(numUAV,1);

states = [x, y, z, theta];

objective = ones(numUAV,1);     % - objective = 1 : the UAV is filled with
                                % water and is going to put out the fire
                                % - objective = 2 : the UAV is empty and is
                                % going to refill
                                % (we assume every one empty at the beginning)

% State Transition Matrix
A = [1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 1];

% Control Matrix
B = @(theta,dt) dt * [cos(theta), 0, 0;
                      sin(theta), 0, 0;
                               0, 1, 0;
                               0, 0, 1];

% Map Parameters
dimgrid = [500 500 500];   % Define the dimensions of the grid

% Fires Parameters

% Fires Positions
x_fire1 = 400;
y_fire1 = 400;
x_fire2 = 450;
y_fire2 = 50;
pos_fire1 = [x_fire1, y_fire1];
pos_fire2 = [x_fire2, y_fire2];

sigma_fire1 = 15;   % Standard deviation of the first fire
                    % (correspond to the extention of the fire)
sigma_fire2 = 15;   % Standard deviation of the second fire
                    % (correspond to the extention of the fire)
inc_threshold1 = 15;  % Distance that has to be reach from the fire 1 
inc_threshold2 = 15;  % Distance that has to be reach from the fire 2

% Water Parameters

% Water Positions
x_water = 50;
y_water = 50;
pos_water = [x_water, y_water];

sigma_water = 60;
wat_threshold = 60;   % Distance that has to be reach from the water source to refill

% Simulation Parameters 
dt = 0.01;
T_sim = 100;

[G_fire,G_water] = objective_density_functions(dimgrid, pos_fire1,pos_fire2,pos_water,sigma_fire1,sigma_fire2,sigma_water,PLOT_DENSITY_FUNCTIONS);

trajectories = zeros(numUAV, 4, T_sim/dt);

% Prepare figure for simulation
figure(2);
colors = lines(numUAV);
hold on;
axis([0 dimgrid(1) 0 dimgrid(2) 0 dimgrid(3)]);
xlabel('X Coordinate');
ylabel('Y Coordinate');
zlabel('Z Coordinate');
title('Simulation');
view(3);

if DO_SIMULATION
    
    count = 0;
    for t = 1:dt:T_sim
        
        count = count+1;
        % Compute the distances from fires and water source for each drone 
        dist_inc1 = pdist2(pos_fire1, states(:,1:2));
        dist_inc2 = pdist2(pos_fire2, states(:,1:2));
        dist_wat = pdist2(pos_water, states(:,1:2));
        
        % Verify if the wanted distance from the target is reached
        for i= 1:numUAV
            if(dist_inc1(i) <= inc_threshold1 || dist_inc2(i) <= inc_threshold2 && objective(i) == 1)
                objective(i) = 2;
            end
            if dist_wat(i) <= wat_threshold && objective(i) == 2
                objective(i) = 1;
            end
        end

        [areas, weigth_centroids, vel] = voronoi_function_FW(numUAV, dimgrid, states, Kp, Ka, Ke, G_fire, G_water, objective);
        
        % Impose a maximum velocity
        vel(:,1) = sign(vel(:,1)) .* min(abs(vel(:,1)), vel_lin_max);
        vel(:,3) = sign(vel(:,3)) .* min(abs(vel(:,3)), vel_ang_max);

        for k = 1:numUAV
            states(k,:) = compute_dynamics(A,B,states(k,:),vel(k,:),dt);
            trajectories(k,:,count) = states(k,:);
        end

        cla;
        
        for i = 1:numUAV
            
            % Plot the current drone position as a marker
            plot3(states(i,1),states(i,2),states(i,3));
            
            drawUnicycle(states(i,1),states(i,2),states(i,4));

        end
        voronoi(states(:,1),states(:,2));

        plot3(x_fire1,y_fire1,0,'x','Color', 'r', 'MarkerSize', sigma_fire1)
        plot3(x_fire2,y_fire2,0,'x','Color', 'r', 'MarkerSize', sigma_fire2)
        plot3(x_water,y_water,0,'o','Color', 'b', 'MarkerSize', sigma_water)

        drawnow;  % Force MATLAB to update the figure

    end

end