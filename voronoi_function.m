function [areas, centroids, vel] = voronoi_function(Map, c_points, kp)
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
    areas = zeros(length(c_x), 1);
    centroids = zeros(length(c_x), 2);
    vel = zeros(length(c_x), 2);

    for i = 1:length(c_x)
        % Estrai i punti della regione assegnata al drone i
        region_points = voronoi_grid(minimum_indices == i, :); % Punti della regione

        % Calcola l'area della regione
        areas(i) = size(region_points, 1);

        % Calcola il centroide della regione
        centroids(i, :) = round(mean(region_points));

        % Calcola il vettore di velocità
        vel(i, :) = kp * (centroids(i, :) - c_points(i, :));
    end

    % Plot della tassellazione di Voronoi
    figure(3)
    imagesc(indices_cell);
    hold on;
    scatter(centroids(:, 1), centroids(:, 2), 60);
    % scatter(c_points(:, 1), c_points(:, 2), 60, 'x');
end