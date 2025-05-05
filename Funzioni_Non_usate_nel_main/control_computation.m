% filepath: c:\Users\jackb\Desktop\Distributed\Distributed_Project\Project\modelSimulation_function.m
function [vel, objective, weigth_centroids] = control_computation(numUAV, dimgrid, states, objective, ...
    LastMeas, count, pos_fire1, pos_est_fire1, pos_fire1_mov, pos_fire2, pos_water, inc_threshold1, inc_threshold2, wat_threshold, ...
    Kp_z, Kp, Ka, Ke, G_fire, G_water, scenario, vel_lin_max,vel_lin_min, vel_lin_z_max, vel_ang_max,t)
    
    % Compute the distances from fires and water source for each drone

    % dist_inc1 = pdist2(pos_fire1, states(:,1:2)); % Distance to the first fire
    dist_inc1 = zeros(numUAV,2);

    dist_inc2 = pdist2(pos_fire2, states(:,1:2)); % Distance to the second fire
    dist_wat  = pdist2(pos_water, states(:,1:2)); % Distance to the water source
    
    % Verify if the wanted distance from the target is reached
    for i = 1:numUAV
        dist_inc1(i) = pdist2(pos_est_fire1(i,:), states(i,1:2));

        % If the drone is close to a fire and its objective is 1 (heading to fire)
        if (dist_inc1(i) <= inc_threshold1 || dist_inc2(i) <= inc_threshold2) && objective(i) == 1
            objective(i) = 2; % Change objective to 2 (heading to refill water)
            pos_est_fire1(i,:) = pos_fire1_mov(t);
            LastMeas = count;
        end

        % If the drone is close to the water source and its objective is 2 (heading to refill)
        if dist_wat(i) <= wat_threshold && objective(i) == 2
            objective(i) = 1; % Change objective to 1 (heading to fire)
        end
    end

    % Compute Voronoi tessellation and velocities
    [areas, weigth_centroids, vel] = voronoi_function_FW(numUAV, dimgrid, states, Kp_z, Kp, Ka, Ke, G_fire, G_water, scenario, objective);

    % Impose a maximum velocity
    % The linear straight velocty has also a minimum velocity since we are considering Fixed wing UAV 
    vel(:,1) = sign(vel(:,1)) .* max(min(abs(vel(:,1)), vel_lin_max), vel_lin_min); % Linear velocity 
    vel(:,2) = sign(vel(:,2)) .* min(abs(vel(:,2)), vel_lin_z_max); % Limit linear velocity along z
    vel(:,3) = sign(vel(:,3)) .* min(abs(vel(:,3)), vel_ang_max); % Limit angular velocity

end