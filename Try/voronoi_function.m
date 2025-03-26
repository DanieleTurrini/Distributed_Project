function [areas, weigth_centroids, acc_des] = voronoi_function(Map, stati, Kp, Ki, Kd, dt, G_fire, G_water, status)
    
    % Crea una griglia di punti con le dimensioni specificate da Map
    [X, Y] = meshgrid(1:Map(1), 1:Map(2));
    voronoi_grid = [X(:), Y(:)];

    c_points = [squeeze(stati(1,1,:)), squeeze(stati(2,1,:))];

    % Estrai le coordinate x e y dei punti iniziali
    c_x = c_points(:,1);
    c_y = c_points(:,2);

    % Calcola le distanze tra i punti iniziali e ciascun punto sulla griglia
    distances = pdist2(voronoi_grid, c_points);

    % Trova l'indice del punto iniziale più vicino per ciascun punto sulla griglia
    [~, minimum_indices] = min(distances, [], 2);

    % Inizializza le aree, i centroidi e i vettori di velocità
    numDrones = length(c_x);
    areas = zeros(numDrones, 1);
    acc_des = zeros(numDrones, 2);
    masses = zeros(numDrones, 1);
    weigth_centroids = zeros(numDrones, 2);

    error = zeros(numDrones,2);
    persistent integral_error;
    if isempty(integral_error)
        integral_error = zeros(numDrones,2);
    end

    persistent previous_error;
    if isempty(previous_error)
        previous_error = zeros(numDrones,2);
    end

    % Calcola le aree e i centroidi pesati per ogni punto
    for i = 1:numDrones
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

        % Errore attuale
        error(i,:) = weigth_centroids(i, :) - c_points(i, :);

        % Componente proporzionale
        P = Kp * error(i,:);

        % Componente Integrale 
        integral_error(i,:) = integral_error(i,:) + error(i,:) * dt; % Accumulo dell'errore
        I = Ki * integral_error(i,:);

        % Componente Derivativa
        D = Kd * (error(i,:) - previous_error(i,:)) / dt; % Differenza tra l'errore attuale e quello precedente
        previous_error(i,:) = error(i,:); % Aggiorno l'errore precedente

        % Calcolo finale dell'accelerazione desiderata
        acc_des(i, :) = P + I + D;
        
    end
end