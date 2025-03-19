function droni = creaDroni_Function(n_points, punti_iniziali, dev_std_gps)
    % Numero di droni
    if nargin < 1
        n_points = 5; % Default
    end

    % Punti iniziali
    if nargin < 2
        punti_iniziali = rand(n_points, 2) * 100; % Default: posizioni casuali in un'area 100x100
    end

    % Inizializzazione della struttura per ogni drone
    for i = 1:n_points
        droni(i).pos_reale = punti_iniziali(i, :);      % Posizione reale (x, y)
        droni(i).pos_gps = droni(i).pos_reale + randn(1,2) * dev_std_gps ; % Aggiunge errore GPS
        droni(i).distanze = [];                         % Sarà calcolata dopo
        droni(i).pos_altri_droni = [];                  % Posizioni degli altri droni
        droni(i).pos_trilat = droni(i).pos_gps;         % Posizione stimata iniziale
        droni(i).inc_trilat = dev_std_gps;              % Posizione stimata iniziale        
        droni(i).pos_sensorFusion = droni(i).pos_gps;   % Posizione stimata iniziale
        droni(i).pos_kal = zeros(1,2);                  % Posizione stimata filtro di Kalman
        droni(i).pos_kal_cov = 100 * eye(2);            % Covarianza iniziale filtro di Kalman 
        droni(i).velocita = [0, 0];                     % Velocità arbitraria (x, y)
        droni(i).status = 2;                            % 1 il drone è carico di acqua e sta andando verso l'incendio -- 2 il drone è scarico di acqua e sta andando a rifornirsi
    end

end