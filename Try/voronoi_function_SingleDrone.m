function [areas, weigth_centroids, w_vel] = voronoi_function_SingleDrone(droni, Map, kp, G_fire, G_water)
    
    c_points = zeros(length(droni), 2);
    % posizione stimata droni
    for i = 1:length(droni)
        c_points(i, :) = droni(i).pos_kal;
    end

    % Controlla se c_points contiene NaN
    if any(isnan(c_points(:)))
        error('c_points contiene valori NaN.');
    end

    % Stato dei droni
    status = [droni.status];

    % Crea una griglia di punti con le dimensioni specificate da Map
    [X, Y] = meshgrid(1:Map(1), 1:Map(2));
    voronoi_grid = [X(:), Y(:)];

    % Calcola le distanze tra i punti iniziali e ciascun punto sulla griglia
    distances = pdist2(voronoi_grid, c_points);

    % Trova l'indice del punto iniziale più vicino per ciascun punto sulla griglia
    [~, minimum_indices] = min(distances, [], 2);

    % Inizializza le aree, i centroidi e i vettori di velocità
    num_points = length(droni);
    areas = zeros(num_points, 1);
    w_vel = zeros(num_points, 2);
    masses = zeros(num_points, 1);
    weigth_centroids = zeros(num_points, 2);

    % Calcola le aree e i centroidi pesati per ogni punto
    for i = 1:num_points
        % Estrai i punti della regione assegnata al drone i
        region_points = voronoi_grid(minimum_indices == i, :); % Punti della regione

        % Controlla se region_points contiene NaN
        if any(isnan(region_points(:)))
            error('region_points contiene valori NaN.');
        end

        % Controlla se region_points è vuoto
        if isempty(region_points)
            disp('Debug Info:');
            disp('voronoi_grid:');
            disp(voronoi_grid);
            disp('minimum_indices:');
            disp(minimum_indices);
            error('region_points è vuoto. Verifica le coordinate in voronoi_grid e minimum_indices.');
        end

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

        % Controlla se weights contiene NaN o se è vettore nullo 
        if isempty(weights)
            error('weights è un vettore vuoto. Verifica le coordinate in region_points.');
        elseif any(isnan(weights(:))) 
            error('weights contiene valori NaN.');
        elseif any(weights < 0)
            error('weights contiene valori negativi.');
        elseif any(weights > 1)
            error('weights contiene valori maggiori di 1.');
        elseif all(weights == 0)
            error('weights contiene solo valori nulli.');
        end

        % Calcolo della massa della regione
        masses(i) = sum(weights);

        % Evita divisioni per zero
        if masses(i) == 0
            error('La massa della regione è zero, impossibile calcolare il centroide pesato.');
        end

        % Calcolo del centroide pesato
        weigth_centroids(i, :) = sum(region_points .* weights, 1) / masses(i);

        % Calcola il vettore di velocità
        w_vel(i, :) = kp * (weigth_centroids(i, :) - c_points(i, :));

        if any(isnan(w_vel(i, :)))
            error('NaN value in w_vel -> Voronoi Function');
        end
    end
end