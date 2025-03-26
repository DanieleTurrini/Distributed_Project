function [areas, weigth_centroids, vel] = voronoi_function_FW(numUAV,dimgrid, states, Kp, Ka, Ke, G_fire, G_water, objective)

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
    distances = zeros(numUAV,1);
    angle_to_goal = zeros(numUAV,1);
    angle_error = zeros(numUAV,1);
    v_lin = zeros(numUAV,1);
    w_ang = zeros(numUAV,1);
    sign_angle = zeros(numUAV,1);

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
        
        % % Calcolo della distanza 
        % distances(i,:) = norm(weigth_centroids(i, :) - states(i,1:2));
        % 
        % % Angolo dal centroide
        % angle_to_goal(i) = atan2(weigth_centroids(i, 2) - states(i,2), ...
        %                            weigth_centroids(i, 1) - states(i,1));
        % 
        % % Angolo di errore
        % angle_error(i) = angle_to_goal(i) - states(i,3); 
        % angle_error(i) = wrapToPi(angle_error(i));
        % 
        % % Segno dell'angolo
        % if angle_error >= 0
        %     sign_angle(i,1) = 1;
        % elseif angle_error < 0
        %     sign_angle(i,1) = -1;
        % end
        % 
        % % Velocità lineare
        % v_lin(i, 1) = Kp * distances(i,1);
        % 
        % 
        % 
        % % Velocità angolare 
        % 
        % if abs(angle_error) <= 0.1  
        %     Ke_c = 0;
        % else
        %     Ke_c = Ke;
        % end
        % 
        % w_ang(i,1) = Ka * (angle_error(i,1) + sign_angle(i) * atan(Ke_c * distances(i,:) / v_lin(i, 1) +1e-5));
        % 
        % % Calcola il vettore di velocità
        % vel(i, :) = [v_lin(i,1),0, w_ang(i,1)];
    end