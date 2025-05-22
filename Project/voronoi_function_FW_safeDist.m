function [areas, weigth_centroids, vel] = voronoi_function_FW_safeDist( ...
    numUAV, dimgrid, states, Kp_z, Kp, Ka, pos_est_fire1, pos_est_fire2, ...
    sigma_est_fire1, sigma_est_fire2, G_water, height_flight, scenario, ...
    objective, initialUAV_pos,delta_safety)

    % Create the grid
    [X, Y] = meshgrid(1:dimgrid(1), 1:dimgrid(2));
    voronoi_grid = [X(:), Y(:)];

    % Distances with virtual points if too close
    all_distances = zeros(size(voronoi_grid,1), numUAV);
    for i = 1:numUAV
        pi = states(i,1:2);
        for j = 1:numUAV
            pj = states(j,1:2);
            if i == j
                all_distances(:, j) = vecnorm(voronoi_grid - pj, 2, 2);
                continue;
            end
            d = norm(pi - pj);
            if d < 2 * delta_safety
                % Compute virtual point
                correction = 2 * (delta_safety - d/2) * (pi - pj) / d;
                pj_virtual = pj + correction;
                all_distances(:, j) = vecnorm(voronoi_grid - pj_virtual, 2, 2);
            else
                all_distances(:, j) = vecnorm(voronoi_grid - pj, 2, 2);
            end
        end
    end

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
        vel(i,:) = UAV_control(weigth_centroids(i, :), states(i,:), Kp, Ka);

        % Altitude Z control
        vel(i,2) = Kp_z * (flight_surface(states(i,1), states(i,2), height_flight, scenario) - states(i,3));
    end
end
