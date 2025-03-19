function droni = multilaterazioneGPS_Function(droni, n_points, dev_std_gps, dev_std_radar)
    %% Miglioramento della stima della posizione tramite multilaterazione
    % La multilaterazione sfrutta le distanze misurate tra il drone di interesse e almeno 3 droni vicini
    % per stimare la posizione con maggiore accuratezza rispetto alla sola posizione GPS.
    
    for i = 1:n_points
        % Ottieni le posizioni GPS degli altri droni e le distanze misurate tramite radar
        pos_altri_droni = droni(i).pos_altri_droni; % Matrice delle posizioni dei droni vicini
        distanze = droni(i).distanze; % Vettore delle distanze misurate tramite radar
        
        % Filtra i droni validi (escludendo distanze nulle, che indicano dati non validi)
        validi = distanze > 0;
        pos_altri_droni = pos_altri_droni(validi, :);
        distanze = distanze(validi);
        
        % Numero di droni rimanenti nel range di misurazione
        n_droni_range = length(distanze);
        
        if n_droni_range >= 3
            % Seleziona i 3 droni più vicini per migliorare la precisione della multilaterazione
            [~, idx] = sort(distanze); % Ordina per distanza crescente
            pos_altri_droni = pos_altri_droni(idx(1:3), :);
            distanze = distanze(idx(1:3));
            
            % Stima iniziale della posizione del drone basata su GPS
            x0 = droni(i).pos_gps(:);
            
            % Definizione della funzione di errore per la multilaterazione
            % Si minimizza la differenza tra le distanze misurate e quelle calcolate dalla posizione stimata
            fun = @(pos_trilat) sqrt(sum((pos_altri_droni - pos_trilat(:)').^2, 2)) - distanze;
            
            try
                % Risoluzione del problema di multilaterazione usando minimi quadrati non lineari
                pos_trilat = lsqnonlin(fun, x0, [], [], optimoptions('lsqnonlin', 'Display', 'off'));
                droni(i).pos_trilat = pos_trilat(:)'; % Salva la posizione stimata
                
                % Calcolo della matrice Jacobiana J della funzione di distanza rispetto alle coordinate del drone
                % Questa matrice descrive la sensibilità delle misure di distanza rispetto alla posizione stimata
                J = [(pos_trilat(1) - pos_altri_droni(:,1)) ./ distanze, ...
                     (pos_trilat(2) - pos_altri_droni(:,2)) ./ distanze];
                
                % Costruzione della matrice di covarianza delle misure di distanza (si assume errore indipendente e costante)
                Sigma_d = diag(dev_std_radar^2 * ones(3,1));
                
                % Calcolo della matrice di covarianza della posizione stimata tramite propagazione dell'errore
                % Si basa sulla relazione Σ_x = (J^T Σ_d^{-1} J)^{-1}
                Sigma_x = inv(J' * inv(Sigma_d) * J);
                
                % Deviazioni standard sulle coordinate stimate
                sigma_x = sqrt(Sigma_x(1,1));
                sigma_y = sqrt(Sigma_x(2,2));
                
                % Calcolo dell'incertezza totale combinata considerando multilaterazione + errore GPS
                % Si assume indipendenza degli errori e si sommano quadraticamente
                droni(i).inc_trilat = sqrt(sigma_x^2 + sigma_y^2 + dev_std_gps^2);
                
            catch ME
                % Se la multilaterazione fallisce, manteniamo la posizione GPS
                warning(['Errore con il drone ', num2str(i), ': ', ME.message]);
                droni(i).pos_trilat = droni(i).pos_gps;
                droni(i).inc_trilat = dev_std_gps; % Assumiamo solo l'errore GPS
            end
        else
            % Se meno di 3 droni sono disponibili nel range, non è possibile applicare la multilaterazione
            % Usiamo solo la posizione GPS con la sua incertezza
            droni(i).pos_trilat = droni(i).pos_gps;
            droni(i).inc_trilat = dev_std_gps;
        end
    end
end
