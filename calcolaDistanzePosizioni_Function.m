
function droni = calcolaDistanzePosizioni_Function(droni, range_sensor, prob_scambio_gps, dev_std_radar)
    n_points = length(droni);
    for i = 1:n_points
        droni(i).distanze = zeros(n_points,2); % Sarà calcolata dopo
        droni(i).pos_altri_droni = zeros(n_points,2); % Posizioni degli altri droni
        for j = 1:n_points
            if i ~= j
                % Calcolo della distanza reale con deviazione standard
                distanza_measured = norm(droni(i).pos_reale - droni(j).pos_reale) + randn * dev_std_radar;
                
                % Verifica se la distanza è entro il range del sensore
                if distanza_measured <= range_sensor
                    % Probabilità di avere il dato effettivo
                    if rand <= prob_scambio_gps
                        droni(i).distanze(j) = distanza_measured;
                        
                        % Posizione GPS con deviazione standard
                        pos_gps = droni(j).pos_gps;
                        droni(i).pos_altri_droni(j, :) = pos_gps;
                    end
                end
            end
        end
    end
end