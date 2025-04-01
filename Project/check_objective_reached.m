function check_objective_reached(pos_fire1, pos_fire2, pos_water, states, inc_threshold1, inc_threshold2, objective) 

    % Compute the distances from fires and water source for each drone
    dist_inc1 = pdist2(pos_fire1, states(:,1:2)); % Distance to the first fire
    dist_inc2 = pdist2(pos_fire2, states(:,1:2)); % Distance to the second fire
    dist_wat  = pdist2(pos_water, states(:,1:2)); % Distance to the water source
    
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