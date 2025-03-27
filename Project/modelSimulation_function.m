% filepath: c:\Users\jackb\Desktop\Distributed\Distributed_Project\Project\modelSimulation_function.m
function [vel, objective] = modelSimulation_function(numUAV, dimgrid, states, objective, ...
    pos_fire1, pos_fire2, pos_water, inc_threshold1, inc_threshold2, wat_threshold, ...
    Kp, Ka, Ke, G_fire, G_water, G_fligt, vel_lin_max, vel_ang_max)
    
%{
%%%% INPUT PARAMETERS:
    -> numUAV - Number of drones (UAVs) in the simulation.
    -> dimgrid - Dimensions of the 3D grid as [length, width, height].
    -> states - Current state of each drone, represented as a [numUAV x 4] matrix.
              Each row contains [x, y, z, theta], where:
              - (x, y, z) are the drone’s coordinates.
              - theta is the drone’s orientation.
    -> objective - Current mission status of each drone [numUAV x 1], where:
                 - 1: The drone is carrying water and heading toward a fire.
                 - 2: The drone is empty and heading to a water source.
    -> pos_fire1 - 2D coordinates [1x2] of the first fire location.
    -> pos_fire2 - 2D coordinates [1x2] of the second fire location.
    -> pos_water - 2D coordinates [1x2] of the water source location.
    -> inc_threshold1 - Distance threshold to consider the first fire "reached".
    -> inc_threshold2 - Distance threshold to consider the second fire "reached".
    -> wat_threshold - Distance threshold to consider the water source "reached".
    -> Kp, Ka, Ke - Proportional gain values for controlling linear and angular velocity.
    -> G_fire - Density function guiding movement toward fire locations.
    -> G_water - Density function guiding movement toward water sources.
    -> G_flight - Density function for the overall flight surface.
    -> vel_lin_max - Maximum allowed linear velocity for drones.
    -> vel_ang_max - Maximum allowed angular velocity for drones.
    -> A - State transition matrix defining drone dynamics.
    -> B - Control matrix function for drone dynamics, depending on theta and dt.
    -> dt - Simulation time step.
    -> trajectories - A 3D array storing the flight paths of all drones over time.
                   Size: [numUAV x 4 x (T_sim/dt)].
    -> count - Current simulation step (iteration index).
%%%% OUTPUT PARAMETERS:
    -> states - Updated state of each drone, maintaining the [numUAV x 4] structure.
    -> trajectories - Updated trajectory data for all drones.
    -> objective - Updated mission status for each drone [numUAV x 1].
 
%}

    % Compute the distances from fires and water source for each drone
    dist_inc1 = pdist2(pos_fire1, states(:,1:2)); % Distance to the first fire
    dist_inc2 = pdist2(pos_fire2, states(:,1:2)); % Distance to the second fire
    dist_wat = pdist2(pos_water, states(:,1:2)); % Distance to the water source
    
    % Verify if the wanted distance from the target is reached
    for i = 1:numUAV
        % If the drone is close to a fire and its objective is 1 (heading to fire)
        if (dist_inc1(i) <= inc_threshold1 || dist_inc2(i) <= inc_threshold2) && objective(i) == 1
            objective(i) = 2; % Change objective to 2 (heading to refill water)
        end
        % If the drone is close to the water source and its objective is 2 (heading to refill)
        if dist_wat(i) <= wat_threshold && objective(i) == 2
            objective(i) = 1; % Change objective to 1 (heading to fire)
        end
    end

    % Compute Voronoi tessellation and velocities
    [areas, weigth_centroids, vel] = voronoi_function_FW(numUAV, dimgrid, states, Kp, Ka, Ke, G_fire, G_water, G_fligt, objective);

    % Impose a maximum velocity
    vel(:,1) = sign(vel(:,1)) .* min(abs(vel(:,1)), vel_lin_max); % Limit linear velocity
    vel(:,3) = sign(vel(:,3)) .* min(abs(vel(:,3)), vel_ang_max); % Limit angular velocity

    % Update states and trajectories

%{
     for k = 1:numUAV
        % Update the state of each drone using the dynamics model
        states(k,:) = compute_dynamics(A, B, states(k,:), vel(k,:), dt);
        % Store the updated state in the trajectories array
        trajectories(k,:,count) = states(k,:);
    end 
%}

end