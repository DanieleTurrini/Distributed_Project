function droni = creaDroni(numDrones, starting_points, dev_std_gps)

    % Inizializzazione della struttura per ogni drone
    for i = 1:numDrones
        droni(i).pos_reale = starting_points(i, :);      % Posizione reale (x, y, z)
        droni(i).pos_gps = droni(i).pos_reale + [randn(1,2) * dev_std_gps, 0]; % Aggiunge errore GPS
        % droni(i).distanze = [];                         % Sarà calcolata dopo
        % droni(i).pos_altri_droni = [];                  % Posizioni degli altri droni
        % droni(i).pos_trilat = droni(i).pos_gps;         % Posizione stimata iniziale
        % droni(i).inc_trilat = dev_std_gps;              % Posizione stimata iniziale        
        % droni(i).pos_sensorFusion = droni(i).pos_gps;   % Posizione stimata iniziale
        droni(i).pos_kal = zeros(1,2);                  % Posizione stimata filtro di Kalman
        droni(i).pos_kal_cov = 100 * eye(2);            % Covarianza iniziale filtro di Kalman 
        droni(i).vel = [0, 0, 0];                       % Velocità iniziale
        droni(i).acc = [0, 0, 0];                       % Accelerazione iniziale 
        droni(i).state = [
                          droni(i).pos_reale(1);
                          droni(i).pos_reale(2);
                          droni(i).pos_reale(3);
                          droni(i).vel(1);
                          droni(i).vel(2);
                          droni(i).vel(3);
                          ];
        droni(i).objective = 2;                            % 1 il drone è carico di acqua e sta andando verso l'incendio -- 2 il drone è scarico di acqua e sta andando a rifornirsi
    end

end