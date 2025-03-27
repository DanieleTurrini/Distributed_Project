function [areas, weigth_centroids, vel] = voronoi_function_FW(numUAV,dimgrid, states, Kp, Ka, Ke, G_fire, G_water, G_fligt, objective)

    % Crea una griglia di punti con le dimensioni specificate da dimgrid
    [X, Y] = meshgrid(1:dimgrid(1), 1:dimgrid(2));
    voronoi_grid = [X(:), Y(:)];

    % Calcola le distanze tra i punti iniziali e ciascun punto sulla griglia
    all_distances = pdist2(voronoi_grid, states(:,1:2));

    % Trova l'indice del punto iniziale più vicino per ciascun punto sulla griglia
    [~, minimum_indices] = min(all_distances, [], 2);

    % Initializations
    areas = zeros(numUAV, 1);
    vel = zeros(numUAV, 3);
    masses = zeros(numUAV, 1);
    weigth_centroids = zeros(numUAV, 2);

    % Calcola le aree e i centroidi pesati per ogni punto
    for i = 1:numUAV

        % Estrai i punti della regione assegnata al drone i
        region_points = voronoi_grid(minimum_indices == i, :); % Punti della regione

        % Calcola l'area della regione
        areas(i) = size(region_points, 1);

        if objective(i) == 1
            % Calcolo della massa della regione
            weights = G_fire(sub2ind(size(G_fire), region_points(:,2), region_points(:,1)));
        elseif objective(i) == 2
            % Calcolo della massa della regione
            weights = G_water(sub2ind(size(G_water), region_points(:,2), region_points(:,1)));
        else
            error('The status variable has an invalid value');
        end

        % Calcolo della massa della regione
        masses(i) = sum(weights);

        % Calcolo del centroide pesato
        weigth_centroids(i, :) = sum(region_points .* weights, 1) / masses(i);

        vel(i,:) = UAV_control(weigth_centroids(i, :),states(i, :), Kp, Ka, Ke);
        
    end