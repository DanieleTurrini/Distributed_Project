clc; clear; close all;

%% Parametri
% Numero di droni
n_droni = 6; % Puoi modificarlo

% Velocita massima
vel_max = 50; % m/s

% Treshold distanze acqua e incendi
fire_trsh = 10; % m
water_trsh = 10; % m

% Colors for plotting
colors = lines(n_droni);

%% Parametri dei sensori
range_sensor = 100; % Range del sensore
prob_dato_effettivo = 0.9; % Probabilità di avere il dato effettivo
prob_gps = 0.9; % Probabilità di avere il dato effettivo GPS
dev_std_radar = 0.0005; % Deviazione standard del radar
dev_std_gps = 0.0001; % Deviazione standard del GPS

% incertezza controllo 
dev_std_control = 0.0001; % Deviazione standard del controllo

%% Creazione dei droni
% Punti iniziali
punti_iniziali = rand(n_droni, 2) * 100; % Posizioni casuali in un'area 100x100

droni = creaDroni_Function(n_droni, punti_iniziali, dev_std_gps); 

% Calcolo delle distanze e delle posizioni degli altri droni
droni = calcolaDistanzePosizioni_Function(droni, range_sensor, prob_dato_effettivo, dev_std_radar);

% Miglioramento della stima della posizione tramite multilaterazione
droni = multilaterazioneGPS_Function(droni, n_droni,  dev_std_gps, dev_std_radar);

%% Sensor fusion
% vediamo se la stima migliora
var_gps = dev_std_gps^2;

stima_pos_sensorFusion = zeros(n_droni, 2);
for i = 1:n_droni
    var_trilat = droni(i).inc_trilat^2;
    % Pesi basati sull'inverso della varianza
    w_gps = 1 / var_gps;
    w_radar = 1 / var_trilat;
    
    % Posizione stimata con sensor fusion
    droni(i).pos_sensorFusion = (w_gps * droni(i).pos_gps + w_radar * droni(i).pos_trilat) / (w_gps + w_radar);
    % la prima volta anche la posizione stimata con il filtro di kalman è uguale
    droni(i).pos_kal = droni(i).pos_sensorFusion;
end

%% Visualizzazione delle stime di posizione con sensor fusion
figure;
hold on;
for i = 1:n_droni
    plot(droni(i).pos_reale(1), droni(i).pos_reale(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', [1, 0.6, 0.6]); % Pastel red
    plot(droni(i).pos_gps(1), droni(i).pos_gps(2), 'bx', 'MarkerSize', 12, 'MarkerFaceColor', [0.6, 0.6, 1]); % Pastel blue
    plot(droni(i).pos_trilat(1), droni(i).pos_trilat(2), 'gx', 'MarkerSize', 12, 'MarkerFaceColor', [0.6, 1, 0.6]); % Pastel green
    plot(droni(i).pos_sensorFusion(1), droni(i).pos_sensorFusion(2), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', [1, 0.6, 1]); % Pastel magenta
    viscircles(droni(i).pos_reale, range_sensor, 'LineStyle', '--', 'LineWidth', 0.25, 'EdgeColor', [0.3, 1, 0.9]); % Pastel
    text(droni(i).pos_reale(1), droni(i).pos_reale(2), sprintf('D.%d', i), 'FontSize', 8);
    text(droni(i).pos_sensorFusion(1), droni(i).pos_sensorFusion(2), sprintf('%d', i), 'FontSize', 6);
end
axis equal;
title('Posizioni dei Droni con Sensor Fusion');
xlabel('X');
ylabel('Y');
grid on;
hold off;
legend('Posizione Reale', 'Posizione GPS', 'Posizione Triangolazione', 'Posizione Sensor Fusion');

%% Mappa VORONOI
dimgrid = [500 500];   % Dimensioni della griglia
kp = 100;               % Costante di proporzionalità Voronoi

%{
 [vx,vy] = voronoi(punti_iniziali(:,1), punti_iniziali(:,2))
figure(2);
hold on;
plot(vx, vy, 'r-', 'LineWidth', 1.5);
axis([0 dimgrid(1) 0 dimgrid(2)]); 
scatter(punti_iniziali(:,1), punti_iniziali(:,2), 100, 'filled');
hold off;
title('Voronoi Tassellation');
 
%}

%% Funzione densità per incendi 

% Definizione dei parametri della densità
pos_fire = [400, 400; 450, 50];
pos_water = [50, 50];

sigma_fire = [40, 15];
sigma_water = 20;
Ampl_inc = [1, 1];
Ampl_inc_prev = Ampl_inc;

% Creazione della griglia di punti
[x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));

% Calcolo della distribuzione gaussiana per incendi e acqua
G_fire = zeros(size(x_m));
for i = 1:size(pos_fire, 1)
    G_fire = G_fire + Ampl_inc(i) * exp(-(((x_m - pos_fire(i, 1)).^2) / (2 * sigma_fire(i)^2) + ((y_m - pos_fire(i, 2)).^2) / (2 * sigma_fire(i)^2)));
end
G_water = exp(-(((x_m - pos_water(1)).^2) / (2 * sigma_water^2) + ((y_m - pos_water(2)).^2) / (2 * sigma_water^2)));

% Inserisci posizione fuochi nella struttura droni
for i = 1:n_droni
    droni(i).pos_fire = pos_fire;
    droni(i).pos_water = pos_water;
end

% Visualizzazione in 3D
figure(3);
surf(x_m, y_m, G_fire);
shading interp; % Per rendere la superficie più liscia
colormap jet;
colorbar;
xlabel('X');
ylabel('Y');
zlabel('Densità');
title('Funzione densità: Incendi');
view(3); % Vista in 3D
drawnow;
hold off;

% status = 1 il drone è carico di acqua e sta andando verso l'incendio
% status = 2 il drone è scarico di acqua e sta andando a rifornirsi 
%% Initial Voronoi Tassellation
[areas,centroids,vel] = voronoi_function_SingleDrone(droni, dimgrid,kp,G_fire,G_water);
INITIAL_VALUES = false;
if INITIAL_VALUES
    sum_areas = sum(areas);
    for i = 1:length(areas)
        fprintf('VORONOI:\n Initial areas %d: %f\n',i,areas(i));
    end
    disp(sum_areas)
    for i = 1:length(areas)
        fprintf('Initial centroids coordinates %d: [%f,%f]\n',i,centroids(i,1),centroids(i,2));
    end
    disp(sum_areas)
end


%% Inizializzazione della simulazione
T_sim = 100; % Durata della simulazione in secondi
dt = 1; % Passo di tempo in secondi

%% Inizializzazione delle traiettorie
traj_reale = zeros(n_droni, 2, T_sim); % Posizioni reali
traj_stimata = zeros(n_droni, 2, T_sim); % Posizioni stimate
traj_misure = zeros(n_droni, 2, T_sim); % Posizioni misurate

% Primo Punto
traj_reale(:,:,1) = punti_iniziali;
traj_stimata(:,:,1) = punti_iniziali;
for i = 1:n_droni
    traj_misure(i,:,1) = droni(i).pos_sensorFusion;
end
%% Simulazione

%{
 disp('Premi un qualsiasi tasto per iniziare la simulazione...');
pause; 
%}


DO_PLOTS = true; % Spostato fuori dal ciclo

for t = 1:dt:T_sim
    
    % Ogni drone Calcola la sua distanza dal fuoco conoscendo la posizione del fuoco la stima della sua posizione
    for i = 1:n_droni
        for j = 1:size(pos_fire, 1)
            dist_to_fire = droni(i).pos_fire(j, :) - droni(i).pos_kal;
            if norm(dist_to_fire) < fire_trsh
                droni(i).status = 1;
                Ampl_inc(j) = Ampl_inc(j) - 0.1;
            end
        end
        dist_to_water = droni(i).pos_water - droni(i).pos_kal;
        if norm(dist_to_water) < water_trsh
            droni(i).status = 2;
        end
    end
    
    %% Voronoi
    % REAL POSITIONS
    nx = zeros(n_droni, 2); % Inizializzazione corretta
    status_ = zeros(n_droni, 1);
    for i = 1:n_droni
        nx(i, :) = droni(i).pos_reale;
        status_(i) =  droni(i).status;
    end
    % Calcola le aree, i centroidi e i vettori di velocità secondo la funzione voronoi_function_SingleDrone
    [areas,centroids,vel] = voronoi_function(dimgrid, nx, kp, G_fire, G_water, status_);
    vel = sign(vel).*min(abs(vel),vel_max);
    for i = 1:n_droni
        droni(i).pos_reale = droni(i).pos_reale + vel(i,:) * dt; % Aggiorna la posizione reale
        % Controllo che la posizione reale non sia fuori dalla mappa
        droni(i).pos_reale(1) = max(1, min(dimgrid(1), droni(i).pos_reale(1)));
        droni(i).pos_reale(2) = max(1, min(dimgrid(2), droni(i).pos_reale(2)));
    end

    % ESTIMATED POSITIONS
    % Calcola le aree, i centroidi e i vettori di velocità secondo la funzione voronoi_function_SingleDrone
    [areas_est,centroids_est,vel_est] = voronoi_function_SingleDrone(droni, dimgrid,kp,G_fire,G_water);
    vel_est = sign(vel_est).*min(abs(vel_est),vel_max) + rand(n_droni, 2) * dev_std_control; % controllo incerto di velocità



    for i = 1:n_droni
        droni(i).velocita = vel_est(i,:);
        if any(isnan(droni(i).velocita))
            disp('Nan values Velocity');
        end
    end
    % Il drone si muove
    
    %% Misure 
    % Arriva un nuovo set di misure ( GPS + trilaterazione + sensor fusion)
    for i = 1:n_droni
        % Calcola le distanze e le posizioni degli altri droni
        droni = calcolaDistanzePosizioni_Function(droni, range_sensor, prob_dato_effettivo, dev_std_radar);
        
        % Migliora la stima della posizione tramite multilaterazione
        droni = multilaterazioneGPS_Function(droni, n_droni,  dev_std_gps, dev_std_radar);
        
        % Sensor fusion
        var_gps = dev_std_gps^2;
        stima_pos_sensorFusion = zeros(n_droni, 2);
        for j = 1:n_droni
            var_trilat = droni(j).inc_trilat ^2; % Corretto l'indice
            % Pesi basati sull'inverso della varianza
            w_gps = 1 / var_gps;
            w_radar = 1 / var_trilat;

            % Posizione stimata con sensor fusion
            droni(j).pos_sensorFusion = (w_gps * droni(j).pos_gps + w_radar * droni(j).pos_trilat) / (w_gps + w_radar);
        end
        traj_misure(i,:,t+1) = droni(i).pos_sensorFusion;
    end 

    %% Kalman
    % Calcola la nuova posizione stimata con il filtro di Kalman
    for i = 1:n_droni
        % Stima della posizione con il filtro di Kalman
        [droni(i).pos_kal, droni(i).pos_kal_cov] = kalmanFilter_SingleDrone_Function(droni(i).pos_kal, droni(i).pos_kal_cov, droni(i).velocita, dev_std_control, dev_std_gps, droni(i).pos_sensorFusion, prob_gps, dt, dimgrid);

        if any(isnan(droni(i).pos_kal))
            error('Nan values in Kalman Filter');
        end
    end

    %% Salvo traiettorie
    for i = 1:n_droni
        traj_reale(i,:,t+1) = droni(i).pos_reale;
        traj_stimata(i,:,t+1) = droni(i).pos_kal;
        traj_misure(i,:,t+1) = droni(i).pos_sensorFusion;
    end

    %% Visualizzazione
    if DO_PLOTS
        % Voronoi stimato 
        figure(4);
        clf;
        hold on;
        axis([0 dimgrid(1) 0 dimgrid(2)]);
        xlabel('X');
        ylabel('Y');
        title('Voronoi Tassellation ESTIMATED');
        for i = 1:n_droni
            
            % Posizioni Droni stimate
            plot(traj_stimata(i,1,t+1), traj_stimata(i,2,t+1), 'o', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));

            % tassellazione voronoi
            voronoi(traj_stimata(:,1,t+1), traj_stimata(:,2,t+1));
        end

        % Posizioni fuochi
        for i = 1:size(pos_fire, 1)
            plot(pos_fire(i, 1), pos_fire(i, 2),'x','Color', 'r', 'MarkerSize', sigma_fire(i)*Ampl_inc(i)+0.05)
        end
        % Posizione acqua
        plot(pos_water(1), pos_water(2),'o','Color', 'b', 'MarkerSize', sigma_water)
        hold off;
        drawnow;

        % Visualizzazione 3D incendi
        for i = 1:size(pos_fire, 1)
            if Ampl_inc_prev(i) ~= Ampl_inc(i)
                figure(5);
                clf;
                surf(x_m, y_m, G_fire);
                shading interp; % Per rendere la superficie più liscia
                colormap jet;
                colorbar;
                xlabel('X');
                ylabel('Y');
                zlabel('Densità');
                title('Funzione densità: Incendi');
                view(3); % Vista in 3D
                drawnow;
                hold off;
            end
        end
        Ampl_inc_prev = Ampl_inc; 
        
    end
end

%% Visualizzazione delle traiettorie
figure(6);
hold on;
for i = 1:n_droni
    plot(squeeze(traj_reale(i,1,:)), squeeze(traj_reale(i,2,:)), 'r-', 'LineWidth', 1.5);
    plot(squeeze(traj_stimata(i,1,:)), squeeze(traj_stimata(i,2,:)), 'b-', 'LineWidth', 1.5);
    plot(squeeze(traj_misure(i,1,:)), squeeze(traj_misure(i,2,:)), 'g-', 'LineWidth', 1.5);
end
axis equal;
title('Traiettorie dei Droni');
xlabel('X');
ylabel('Y');
grid on;
hold off;
legend('Posizione Reale', 'Posizione Stimata', 'Posizione Misurata');