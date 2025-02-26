clc; clear; close all;

% Parametri
dim_mappa = [100, 100]; % Dimensioni della griglia (100x100)
kp = 0.2;               % Parametro di guadagno per il controllo di Lloyd
n_iter = 20;            % Numero massimo di iterazioni
raggio_raggiungimento = 1; % Raggio per considerare l'incendio raggiunto

% Numero fisso di incendi e aerei
num_incendi = 3;
num_aerei = 3;

% Creazione della griglia della mappa
[X, Y] = meshgrid(1:dim_mappa(1), 1:dim_mappa(2));
voronoi_grid = [X(:), Y(:)]; % Punti della mappa

% Definizione delle posizioni casuali degli incendi e delle loro ampiezze
incendi = randi([30, 90], num_incendi, 2);
ampiezze_incendi = randi([5, 20], num_incendi, 1);

% Inizializzazione aerei in punti lontani dagli incendi
sp_aerei = zeros(num_aerei, 2);
for i = 1:num_aerei
    while true
        candidato = randi([0, 10], 1, 2);
        if min(vecnorm(candidato - incendi, 2, 2)) > 15 % Deve stare lontano da tutti gli incendi
            sp_aerei(i, :) = candidato;
            break;
        end
    end
end

% Memorizzazione delle traiettorie
traiettorie_x = zeros(n_iter, num_aerei);
traiettorie_y = zeros(n_iter, num_aerei);

figure(1);
hold on;
for iter = 1:n_iter
    clf;

    % Salvataggio delle posizioni attuali degli aerei
    traiettorie_x(iter, :) = sp_aerei(:, 1);
    traiettorie_y(iter, :) = sp_aerei(:, 2);
    
    % Calcolo della Voronoi
    distanze = pdist2(voronoi_grid, sp_aerei);
    [~, min_idx] = min(distanze, [], 2);
    regioni = reshape(min_idx, dim_mappa);
    
    % Plot della Voronoi
    imagesc(regioni);
    colormap(parula);
    hold on;
    
    % Aggiornamento posizioni aerei
    for i = 1:num_aerei
        % Ottieni l'incendio assegnato all'aereo
        idx_incendio = find(min_idx == i);
        
        if ~isempty(idx_incendio)
            centroide = incendi(i, :); % Centroide della regione assegnata

            % Calcola la distanza dall'incendio
            distanza = norm(sp_aerei(i, :) - centroide);
            
            % Muove l'aereo verso il centroide solo se non è già arrivato
            if distanza > raggio_raggiungimento
                sp_aerei(i, :) = sp_aerei(i, :) + kp * (centroide - sp_aerei(i, :));
            end
            
            % Plot degli aerei
            plot(sp_aerei(i, 2), sp_aerei(i, 1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'w');
            
            % Plot incendi
            plot(centroide(2), centroide(1), 'rx', 'MarkerSize', 10 + ampiezze_incendi(i) / 2, 'LineWidth', 2);
        end
    end
    
    pause(0.5);
end
hold off;

% FIGURA SEPARATA: Plotta le traiettorie degli aerei
figure(2);
hold on;
title('Traiettorie degli aerei verso gli incendi');
xlabel('X');
ylabel('Y');
grid on;

for i = 1:num_aerei
    plot(traiettorie_y(:, i), traiettorie_x(:, i), '-o', 'LineWidth', 1.5, 'MarkerSize', 5);
end

legend(arrayfun(@(x) sprintf('Aereo %d', x), 1:num_aerei, 'UniformOutput', false));
hold off;
