function [areas, weigth_centroids, w_vel] = voronoi_function(Map, c_points, kp, G_fire, G_water, status)
    % Crea una griglia di punti con le dimensioni specificate da Map
    [X, Y] = meshgrid(1:Map(1), 1:Map(2));
    voronoi_grid = [X(:), Y(:)];

    % Estrai le coordinate x e y dei punti iniziali
    c_x = c_points(:, 1);
    c_y = c_points(:, 2);

    % Calcola le distanze tra i punti iniziali e ciascun punto sulla griglia
    distances = pdist2(voronoi_grid, c_points);

    % Trova l'indice del punto iniziale più vicino per ciascun punto sulla griglia
    [~, minimum_indices] = min(distances, [], 2);

    % Assegna l'indice a ciascun punto sulla griglia
    indices_cell = reshape(minimum_indices, Map);

    % Inizializza le aree, i centroidi e i vettori di velocità
    num_points = length(c_x);
    areas = zeros(num_points, 1);
    centroids = zeros(num_points, 2);
    w_vel = zeros(num_points, 2);
    masses = zeros(num_points, 1);
    weigth_centroids = zeros(num_points, 2);

    % Calcola le aree e i centroidi pesati per ogni punto
    for i = 1:num_points
        % Estrai i punti della regione assegnata al drone i
        region_points = voronoi_grid(minimum_indices == i, :); % Punti della regione

        % Calcola l'area della regione
        areas(i) = size(region_points, 1);

        if status(i) == 1
            % Calcolo della massa della regione
            weights = G_fire(sub2ind(size(G_fire), region_points(:,2), region_points(:,1)));
        elseif status(i) == 2
            % Calcolo della massa della regione
            weights = G_water(sub2ind(size(G_water), region_points(:,2), region_points(:,1)));
        else
            error('The status variable has an invalid value');
        end

        % Calcolo della massa della regione
        masses(i) = sum(weights);

        % Calcolo del centroide pesato
        weigth_centroids(i, :) = sum(region_points .* weights, 1) / masses(i);

        % Calcola il vettore di velocità
        w_vel(i, :) = kp * (weigth_centroids(i, :) - c_points(i, :));
    end
end