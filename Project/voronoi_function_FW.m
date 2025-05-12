function [areas, weigth_centroids, vel] = voronoi_function_FW( numUAV, dimgrid, states, Kp_z, Kp, Ka, pos_est_fire1, pos_est_fire2, sigma_est_fire1, sigma_est_fire2, G_water,height_flight, scenario, objective, initialUAV_pos)

    % Create a grid of points with dimensions specified by dimgrid
    [X, Y] = meshgrid(1:dimgrid(1), 1:dimgrid(2));
    voronoi_grid = [X(:), Y(:)];

    % Compute distances between initial points and each point on the grid
    all_distances = pdist2(voronoi_grid, states(:,1:2));

    % Find the index of the closest initial point for each point on the grid
    [~, minimum_indices] = min(all_distances, [], 2);

    % Initializations
    areas = zeros(numUAV, 1);
    vel = zeros(numUAV, 3);
    masses = zeros(numUAV, 1);
    weigth_centroids = zeros(numUAV, 2);

    % Compute areas and weighted centroids for each UAV
    for i = 1:numUAV

        G_fire = fires_dens_function(dimgrid, pos_est_fire1(i,:), pos_est_fire2(i,:), sigma_est_fire1(i,1), sigma_est_fire2(i,1)) ;
        
        % Extract the points of the region assigned to UAV i
        region_points = voronoi_grid(minimum_indices == i, :); % Points of the region

        % Compute the area of the region
        areas(i) = size(region_points, 1);

        if objective(i) == 1
            % Compute the mass of the region
            weights = G_fire(sub2ind(size(G_fire), region_points(:,2), region_points(:,1)));
        elseif objective(i) == 2
            % Compute the mass of the region
            weights = G_water(sub2ind(size(G_water), region_points(:,2), region_points(:,1)));
        elseif objective(i) == 3
            % Compute the mass of the region
            %weights = exp(-vecnorm(region_points - initialUAV_pos(i, 1:2), 2, 2));
            sigma = 20; 
            weights = exp( -sum((region_points - initialUAV_pos(i,1:2)).^2, 2) / (2 * sigma^2) );


        else
            error('The status variable has an invalid value');
        end

        % Compute the mass of the region
        masses(i) = sum(weights);
        if masses(i) == 0
            masses(i) = 1; % Avoid division by zero
        end

        % Compute the weighted centroid
        weigth_centroids(i, :) = sum(region_points .* weights, 1) / masses(i);
        
        % Control velocity on the x-y plane
        vel(i,:) = UAV_control(weigth_centroids(i, :),states(i, :), Kp, Ka);

        % Control velocity on z
        vel(i,2) = Kp_z * (flight_surface(states(i,1), states(i,2), height_flight, scenario) - states(i,3));
        
    end
    
end
