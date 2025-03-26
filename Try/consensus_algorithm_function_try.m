function [nx_Est_new, P_new] = consensus_algorithm_function_try(nx_Est, P, adjacency_matrix, alpha)
    % Creiamo una copia temporanea delle stime e delle covarianze
    nx_Est_new = nx_Est;
    P_new = P;

    % Numero di droni
    numPoints = size(nx_Est, 1);

    % Aggiorniamo le stime con il consenso ponderato
    for i = 1:numPoints
        neighbors = find(adjacency_matrix(i, :) == 1);  % Trova i droni connessi a "i"
        if ~isempty(neighbors)
            % Calcola la media ponderata delle stime dei vicini
            weights = zeros(length(neighbors), 1);
            for k = 1:length(neighbors)
                j = neighbors(k);
                weights(k) = 1 / trace(P(:,:,j));
            end
            weights = weights / sum(weights);  % Normalizza i pesi

            consensus_pos = zeros(1, 2);
            consensus_cov = zeros(2, 2);
            for k = 1:length(neighbors)
                j = neighbors(k);
                consensus_pos = consensus_pos + weights(k) * nx_Est(j, :);
                consensus_cov = consensus_cov + weights(k) * P(:,:,j);
            end

            % Aggiorna la stima e la covarianza con il consenso ponderato
            nx_Est_new(i, :) = (1 - alpha) * nx_Est(i, :) + alpha * consensus_pos;
            P_new(:,:,i) = (1 - alpha) * P(:,:,i) + alpha * consensus_cov;
        end
    end
end