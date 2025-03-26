clc; clear; close all;

% Numero di robot
numDrones = 3;

% Stato dei robot [x, y, theta]
stati = rand(numDrones, 3) * 10; % Posizioni iniziali casuali in un'area 10x10

% Velocità massime
v_max = 1.0;
w_max = 1.0;

% Guadagni di controllo
Kp = 0.5;  % Guadagno per la velocità lineare
Ka = 1.0;  % Guadagno per la velocità angolare

% Parametri della simulazione
dt = 0.1; % Passo di tempo
steps = 200; % Numero di iterazioni

% Posizione iniziale del target
goal = [5, 5];

% Generazione colori per ogni robot
colors = lines(numDrones);

% Creazione della figura
figure; hold on;
axis([0 10 0 10]);
grid on;

for t = 1:steps
    clf; hold on;
    axis([0 10 0 10]);
    grid on;
    title(sprintf('Step: %d', t));
    
    % Muovi il target lungo un percorso circolare
    goal(1) = 5 + 3 * cos(0.02 * pi * t);
    goal(2) = 5 + 3 * sin(0.02 * pi * t);
    plot(goal(1), goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    
    for i = 1:numDrones
        % Calcolo della distanza e angolo verso il goal
        dx = goal(1) - stati(i,1);
        dy = goal(2) - stati(i,2);
        distance = hypot(dx, dy);
        angle_to_goal = atan2(dy, dx);
        angle_error = angle_to_goal - stati(i,3);

        % Normalizzazione dell'errore angolare in [-pi, pi]
        angle_error = mod(angle_error + pi, 2*pi) - pi;

        % Calcolo delle velocità
        v = Kp * distance;
        w = Ka * angle_error;

        % Limitazione delle velocità
        v = sign(v) * min(abs(v), v_max);
        w = sign(w) * min(abs(w), w_max);

        % Aggiornamento dello stato
        stati(i,1) = stati(i,1) + v * cos(stati(i,3)) * dt;
        stati(i,2) = stati(i,2) + v * sin(stati(i,3)) * dt;
        stati(i,3) = stati(i,3) + w * dt;

        % Disegno della posizione
        plot(stati(i,1), stati(i,2), 'o', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));

        % Disegno dell'orientamento come triangolo
        triangle_length = 0.5;
        triangle_width = 0.3;

        vertices = [0, triangle_width/2; 0, -triangle_width/2; triangle_length, 0];
        R = [cos(stati(i,3)), -sin(stati(i,3)); sin(stati(i,3)), cos(stati(i,3))];
        rotated_vertices = (R * vertices')';
        rotated_vertices(:,1) = rotated_vertices(:,1) + stati(i,1);
        rotated_vertices(:,2) = rotated_vertices(:,2) + stati(i,2);
        rotated_vertices = [rotated_vertices; rotated_vertices(1,:)];
        plot(rotated_vertices(:,1), rotated_vertices(:,2), '-', 'Color', colors(i,:), 'LineWidth', 2);
    end

    pause(0.05); % Ritardo per visualizzazione
end
