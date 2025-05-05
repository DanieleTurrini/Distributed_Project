function check_objective_reached(numUAV, t, pos_fire1_mov, pos_fire2, pos_water, pos_est_fire1, sigma_est_fire1,sigma_fire1_mov, Qc, states, states_est, LastMeas, inc_threshold1, inc_threshold2, wat_threshold, sensor_range, objective, count) 

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
            inc_threshold1(i) = sigma_est_fire1(i,1);

            if dist_real_inc1(i) <= sensor_range && objective(i) == 1

                pos_est_fire1(i,:) = pos_fire1_mov(t) + 10 * rand(1,1);
                sigma_est_fire1(i,1) = sigma_fire1_mov(t) + 2 * rand(1,1);
                LastMeas(i) = count;
                
                % Definition of Q
                for j = 1:numUAV
                    if j ~= i
                        Qc(:,j) = 2*1/LastMeas(j) + 0.2 * rand(1,numUAV);
                    end
                end
                
                for s = 1:numUAV
                    Qc(s,i) = 1 - (sum(Qc(s,:)) - Qc(s,i));
                end

            end
            
            % If the drone is close to a fire and its objective is 1 (heading to fire)
            if dist_inc1(i) <= inc_threshold1(i) && objective(i) == 1

                objective(i) = 2; % Change objective to 2 (heading to refill water)

            elseif dist_inc2(i) <= inc_threshold2 && objective(i) == 1

                objective(i) = 2; % Change objective to 2 (heading to refill water)

            end

            % If the drone is close to the water source and its objective is 2 (heading to refill)
            if dist_wat(i) <= wat_threshold && objective(i) == 2

                objective(i) = 1; % Change objective to 1 (heading to fire)

            end
        end