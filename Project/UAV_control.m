function vel = UAV_control(weigth_centroids, state, Kp, Ka, Ke)
    
    sign_angle = 0;
    % Calcolo della distanza 
    distances = norm(weigth_centroids - state(1:2));

    % Angolo dal centroide
    angle_to_goal = atan2(weigth_centroids(2) - state(2), ...
                          weigth_centroids(1) - state(1));
    
    % Angolo di errore
    angle_error = angle_to_goal - state(4); 
    angle_error = wrapToPi(angle_error);
    
    % Segno dell'angolo
    if angle_error >= 0
        sign_angle = 1;
    elseif angle_error < 0
        sign_angle = -1;
    end
    
    % Velocità lineare
    v_lin = Kp * distances;

    % Velocità angolare 
    if abs(angle_error) <= 0.1  
        Ke_c = 0;
    else
        Ke_c = Ke;
    end

    w_ang = Ka * (angle_error + sign_angle * atan(Ke_c * distances / v_lin + 1e-5));

    % Calcola il vettore di velocità
    vel = [v_lin, 0, w_ang];
    