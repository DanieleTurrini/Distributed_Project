% voronoi_function_FW computes the Voronoi partitioning for a set of UAVs over a grid,
% calculates the area and weighted centroid of each UAV's region based on different objectives, 
% using a safe parameter to avoid collisions
% and determines the control velocities for each UAV.

function [areas, weigth_centroids, vel, virtual_pos] = voronoi_function_FW_safeDist( ...
    numUAV, dimgrid, states_est, Kp_z, Kp, Ka, pos_est_fire1, pos_est_fire2, ...
    sigma_est_fire1, sigma_est_fire2, G_water, height_flight, scenario, ...
    objective, initialUAV_pos,delta_safety)

    % Create the grid
    [X, Y] = meshgrid(1:dimgrid(1), 1:dimgrid(2));
    voronoi_grid = [X(:), Y(:)];

    % Distances with virtual points if too close
    all_distances = zeros(size(voronoi_grid,1), numUAV);
    all_corrections = zeros(numUAV,2);
    virtual_pos = zeros(numUAV,2);
    distances = zeros(numUAV,numUAV);
    min_distances = zeros(numUAV,1);

    for i = 1:numUAV
        pi = states_est(i,1:2);

        virtual_pos(i,1:2) = states_est(i,1:2);

        for j = 1:numUAV
            pj = states_est(j,1:2);

            if i == j 
                distances(i,j) = +Inf; 
                continue;
            end

            distances(i,j) = norm(pi - pj);
        end  
    end
    
    for i = 1:numUAV
        [min_distances(i),j] = min(distances(i,:));

        if min_distances(i) < 2*delta_safety


            pi = states_est(i,1:2);
            pj = states_est(j,1:2);

            d = distances(i,j);

            correction = 2 * (delta_safety - d/2) * (pi - pj) / d;

            correction(1) = sign(correction(1)) .* min(abs(correction(1)), d/2 - 3);
            correction(2) = sign(correction(2)) .* min(abs(correction(2)), d/2 - 3);

            virtual_pos(j,:) = virtual_pos(j,:) + correction;
            
        end

    end

    all_distances = pdist2(voronoi_grid, virtual_pos);

    % Voronoi cell assignment
    [~, minimum_indices] = min(all_distances, [], 2);

    % Initializations
    areas = zeros(numUAV, 1);
    vel = zeros(numUAV, 3);
    masses = zeros(numUAV, 1);
    weigth_centroids = zeros(numUAV, 2);

    % Loop for each UAV
    for i = 1:numUAV

        % Objective density (e.g., fire or water)
        G_fire = fires_dens_function(dimgrid, pos_est_fire1(i,:), pos_est_fire2(i,:), ...
                                     sigma_est_fire1(i,1), sigma_est_fire2(i,1));

        % Extract the points of the region assigned to UAV i
        region_points = voronoi_grid(minimum_indices == i, :);
        areas(i) = size(region_points, 1);

        % Compute weights
        switch objective(i)
            case 1
                weights = G_fire(sub2ind(size(G_fire), region_points(:,2), region_points(:,1)));
            case 2
                weights = G_water(sub2ind(size(G_water), region_points(:,2), region_points(:,1)));
            case 3
                sigma = 10;
                weights = exp(-sum((region_points - initialUAV_pos(i,1:2)).^2, 2) / (2 * sigma^2));
            otherwise
                error('The status variable has an invalid value');
        end

        % Mass of the region and weighted centroid
        masses(i) = sum(weights);
        if masses(i) == 0
            masses(i) = 1; % to avoid division by zero
        end
        weigth_centroids(i, :) = sum(region_points .* weights, 1) / masses(i);

        % XY plane control
        vel(i,:) = UAV_control(weigth_centroids(i, :), states_est(i,:), Kp, Ka);

        % Altitude Z control
        vel(i,2) = Kp_z * (flight_surface(states_est(i,1), states_est(i,2), height_flight, scenario) - states_est(i,3));
    end
end
