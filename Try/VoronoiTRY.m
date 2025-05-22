function [areas, weigth_centroids, vel] = voronoi_function_FW( ...
    numUAV, dimgrid, states, Kp_z, Kp, Ka, pos_est_fire1, pos_est_fire2, ...
    sigma_est_fire1, sigma_est_fire2, G_water, height_flight, scenario, ...
    objective, initialUAV_pos)

    % Parametri di sicurezza
    delta_safety = 10; % distanza minima (in pixel sulla griglia)

    % Crea la griglia
    [X, Y] = meshgrid(1:dimgrid(1), 1:dimgrid(2));
    voronoi_grid = [X(:), Y(:)];

    % Distanze con punti virtuali se troppo vicini
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
                % Calcolo punto virtuale
                correction = 2 * (delta_safety - d/2) * (pi - pj) / d;
                pj_virtual = pj + correction;
                all_distances(:, j) = vecnorm(voronoi_grid - pj_virtual, 2, 2);
            else
                all_distances(:, j) = vecnorm(voronoi_grid - pj, 2, 2);
            end
        end
    end

    % Assegnazione della cella Voronoi
    [~, minimum_indices] = min(all_distances, [], 2);

    % Inizializzazioni
    areas = zeros(numUAV, 1);
    vel = zeros(numUAV, 3);
    masses = zeros(numUAV, 1);
    weigth_centroids = zeros(numUAV, 2);

    % Ciclo per UAV
    for i = 1:numUAV

        % Densità obiettivo (es. fuoco o acqua)
        G_fire = fires_dens_function(dimgrid, pos_est_fire1(i,:), pos_est_fire2(i,:), ...
                                     sigma_est_fire1(i,1), sigma_est_fire2(i,1));

        % Estrai i punti della regione assegnata a UAV i
        region_points = voronoi_grid(minimum_indices == i, :);
        areas(i) = size(region_points, 1);

        % Calcolo pesi
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

        % Massa della regione e centroide pesato
        masses(i) = sum(weights);
        if masses(i) == 0
            masses(i) = 1; % per evitare divisione per zero
        end
        weigth_centroids(i, :) = sum(region_points .* weights, 1) / masses(i);

        % Controllo piano XY
        vel(i,:) = UAV_control(weigth_centroids(i, :), states(i,:), Kp, Ka);

        % Controllo altitudine Z
        vel(i,2) = Kp_z * (flight_surface(states(i,1), states(i,2), height_flight, scenario) - states(i,3));
    end
end
