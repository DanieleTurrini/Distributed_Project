function [areas, weigth_centroids, acc_des] = voronoi_function_3D(droni, Map, Kp, Ki, Kd, dt, G_fire, G_water)

    numDrones = length(droni);
    % Crea una griglia di punti con le dimensioni specificate da Map
    [X, Y] = meshgrid(1:Map(1), 1:Map(2));
    voronoi_grid = [X(:), Y(:)];

    % Calcola le distanze tra i punti iniziali e ciascun punto sulla griglia
    curr_pos = zeros(numDrones,3);
    for i =1:numDrones
        curr_pos(i,:) = droni(i).pos_reale;
    end

    distances = pdist2(voronoi_grid, curr_pos(:,1:2));

    % Trova l'indice del punto iniziale più vicino per ciascun punto sulla griglia
    [~, minimum_indices] = min(distances, [], 2);

    % Inizializza le aree, i centroidi e i vettori di velocità
    areas = zeros(numDrones, 1);
    acc_des = zeros(numDrones, 3);
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

    % Calcola le aree e i centroidi pesati per ogni drone
    for i = 1:numDrones
        % Estrai i punti della regione assegnata al drone i
        region_points = voronoi_grid(minimum_indices == i, :); % Punti della regione

        % Calcola l'area della regione
        areas(i) = size(region_points, 1);

        if droni(i).objective == 1
            % Calcolo della massa della regione
            weights = G_fire(sub2ind(size(G_fire), region_points(:,2), region_points(:,1)));
        elseif droni(i).objective == 2
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
        error(i,:) = weigth_centroids(i, :) - curr_pos(i, 1:2);

        % Componente proporzionale
        P = Kp * error(i,:);

        % Componente Integrale 
        integral_error(i,:) = integral_error(i,:) + error(i,:) * dt; % Accumulo dell'errore
        I = Ki * integral_error(i,:);

        % Componente Derivativa
        D = Kd * (error(i,:) - previous_error(i,:)) / dt; % Differenza tra l'errore attuale e quello precedente
        previous_error(i,:) = error(i,:); % Aggiorno l'errore precedente

        % Calcolo finale dell'accelerazione desiderata
        acc_des(i, 1:2) = P + I + D;
        acc_des(i,3) = 0;

    end
end
