clear
close all
clc
%% Filtro di Kalman Distribuito

% Il codice simula un sistema distribuito
% in cui due veicoli si muovono con dinamiche semplici e vengono monitorati da sensori rumorosi.
% Il filtro di Kalman viene utilizzato per stimare la posizione dei veicoli
% combinando misure di GPS e radar.

%% Inizializzazione Sistema

% Parametri generali
Dt = 0.1; % Passo di campionamento (100 ms)
t = 0:Dt:100; % Intervallo di simulazione

%% Inizializzazione Veicolo 1
x1 = rand(2,1); % Posizione iniziale casuale
% Comando di controllo (accelerazione variabile nel tempo)
ux1 = 1 + sin(t);
uy1 = t + sin(t*5);
u1 = [ux1;uy1];

%% Inizializzazione Veicolo 2
x2 = rand(2,1); % Posizione iniziale casuale
% Comando di controllo (accelerazione variabile nel tempo)
ux2 = 1 + cos(t);
uy2 =  1 + cos(t/5);
u2 = [ux2;uy2];

%% Attrito & Dinamica
% Modello:
% x_i+1 = a⋅x_i + b⋅u_i
% posizione futura = effetto attrito + effetto controllo

A = eye(2) * -0.1;  % Matrice di transizione stato (attrito)
B = eye(2) * Dt * 1.2; % Matrice di controllo

%% Simulazione della dinamica dei veicoli​
x1Store = zeros(2, length(t));
x1Store(:,1) = x1;
x2Store = zeros(2, length(t));
x2Store(:,1) = x2;

for i = 1:length(t)-1
    % Aggiornamento della posizione del veicolo 1 -> ideale
    x1Store(:,i+1) = A * x1Store(:,i) + B * u1(:,i);
    % Aggiornamento della posizione del veicolo 2 -> ideale
    x2Store(:,i+1) = A * x2Store(:,i) + B * u2(:,i);
end

%% Incertezze misure

% Errore nelle misure di accelerazione
sigma_u1 = 0.1;
u1_bar = u1 + randn(2, length(u1)) * sigma_u1;
sigma_u2 = 0.05;
u2_bar = u2 + randn(2, length(u2)) * sigma_u2;

% Errore nelle misure GPS
sigma_gps1 = 0.3;
x1GPS = x1Store + randn(2, length(x1Store)) .* sigma_gps1;
ProbGPS1 = 0.05; % Probabilità che il GPS sia disponibile

sigma_gps2 = 0.2;
x2GPS = x2Store + randn(2, length(x2Store)) .* sigma_gps2;
ProbGPS2 = 0.9;

% Errore nelle misure Radar (posizione relativa veicoli)
sigma_radar = 0.1;
ProbRadar = 0.8;

%% Inizializzazione del Filtro di Kalman

% ------> Veicolo 1 <------
x1Est = zeros(2, length(t));

% covarianza errore di stima 
P1 = 100 * eye(2); % Matrice 2×2
P1Store = zeros(2, 2, length(t));
P1PredStore = zeros(2, 2, length(t));

% ----> Veicolo 2 <------
x2Est = zeros(2, length(t));

% covarianza errore di stima 
P2 = 100 * eye(2); % Matrice 2×2 
P2Store = zeros(2, 2, length(t));
P2PredStore = zeros(2, 2, length(t));

for i = 1:length(t)-1

    %% Filtro di Kalman per il veicolo 1
    
    % PREDIZIONE
    x1EstPred = A * x1Est(:,i) + B * u1_bar(:,i);
    P1pred = A * P1 * A' + B * sigma_u1^2 * B'; % predizione covarianza dell'errore
    
    % Aggiornamento basato su GPS
    if rand(1) <= ProbGPS1
        H = eye(2); % matrice di osservazione
        R = sigma_gps1^2 * eye(2); % covarianza dell'errore GPS
        InnCov = H * P1pred * H' + R;
        W = P1pred * H' / InnCov;
        x1Est(:,i+1) = x1EstPred + W * (x1GPS(:,i+1) - H * x1EstPred);
        P1 = (eye(2) - W * H) * P1pred;
    else
        x1Est(:,i+1) = x1EstPred;
        P1 = P1pred;
    end

    P1Store(:,:,i+1) = P1;  
    P1PredStore(:,:,i+1) = P1pred; 

    %% Filtro di Kalman per il veicolo 2
    
    % PREDIZIONE
    x2EstPred = A * x2Est(:,i) + B * u2_bar(:,i);
    P2pred = A * P2 * A' + B * sigma_u2^2 * B'; % predizione covarianza dell'errore
    
    % Aggiornamento basato su GPS
    if rand(1) <= ProbGPS2
        H = eye(2); 
        R = sigma_gps2^2 * eye(2); % covarianza dell'errore GPS
        InnCov = H * P2pred * H' + R;
        W = P2pred * H' / InnCov;
        x2Est(:,i+1) = x2EstPred + W * (x2GPS(:,i+1) - H * x2EstPred);
        P2 = (eye(2) - W * H) * P2pred;
    else
        x2Est(:,i+1) = x2EstPred;
        P2 = P2pred;
    end

    P2Store(:,:,i+1) = P2;  
    P2PredStore(:,:,i+1) = P2pred;

end

%% Visualizzazione dei risultati
PLOTPos = true;
PLOTCov = false;
PLOTAutoc = false;
PLOTHist = false;

%% Posizioni
if PLOTPos == true
    figure(1), clf, hold on;

    % Primo grafico: Posizione X reale vs stimata per il veicolo 1
    subplot(2, 2, 1);  
    plot(t, x1Store(1,:)); % Posizione X reale
    hold on;
    plot(t, x1Est(1,:));   % Posizione X stimata
    title('Posizione X reale vs stimata per il veicolo 1');
    xlabel('Tempo [s]');
    ylabel('Posizione X');
    legend('x1 Reale', 'x1 Stimato');
    grid on; 
    % Secondo grafico: Posizione Y reale vs stimata per il veicolo 1
    subplot(2, 2, 2); 
    plot(t, x1Store(2,:));  % Posizione Y reale
    hold on;
    plot(t, x1Est(2,:));    % Posizione Y stimata
    title('Posizione Y reale vs stimata per il veicolo 1');
    xlabel('Tempo [s]');
    ylabel('Posizione Y');
    legend('y1 Reale', 'y1 Stimato');
    grid on;

    % Posizione X reale vs stimata per il veicolo 2
    subplot(2, 2, 3);  
    plot(t, x2Store(1,:)); % Posizione X reale
    hold on;
    plot(t, x2Est(1,:));   % Posizione X stimata
    title('Posizione X reale vs stimata per il veicolo 2');
    xlabel('Tempo [s]');
    ylabel('Posizione X');
    legend('x2 Reale', 'x2 Stimato');
    grid on;   
    % Secondo grafico: Posizione Y reale vs stimata per il veicolo 2
    subplot(2, 2, 4); 
    plot(t, x2Store(2,:));  % Posizione Y reale
    hold on;
    plot(t, x2Est(2,:));    % Posizione Y stimata
    title('Posizione Y reale vs stimata per il veicolo 2');
    xlabel('Tempo [s]');
    ylabel('Posizione Y');
    legend('y2 Reale', 'y2 Stimato');
    grid on;


    % Plot 3D per il veicolo 1
    figure(2);
    %t = linspace(0, 10, 1001);             % Tempo da 0 a 10 secondi
    x1Store = [x1Store(1,:); x1Store(2,:)];             % Posizione X e Y (ad esempio una traiettoria circolare)
    plot3(t,x1Store(1,:), x1Store(2,:));   % Posizione reale
    ylabel('Posizione X');
    zlabel('Posizione Y');
    xlabel('Tempo [s]');
    grid on;
    title('Posizione spaziale 3D del veicolo 1');



end

%% Covarianza
if PLOTCov == true
    % Andamento della covarianza
    figure(3), clf, hold on;
    plot(t, squeeze(P1Store(1,1,:)), 'b'); % Covarianza x1
    plot(t, squeeze(P1PredStore(1,1,:)), 'r--'); % Covarianza predetta x1
    plot(t, squeeze(P2Store(1,1,:)), 'k--'); % Covarianza x2
    plot(t, squeeze(P2PredStore(1,1,:)), 'g--'); % Covarianza predetta x2
    legend('P1', 'P1 Predetto', 'P2', 'P2 Predetto');
    xlabel('Tempo [s]');
    ylabel('Covarianza');
    set(gca, 'YScale', 'log');
    title('Andamento della covarianza di errore');
end

%% Autocorrelazione
if PLOTAutoc == true
    % Autocorrelazione delle misure
    figure(4), clf, hold on;
    subplot(2,1,1)
    autocorr(u1_bar(1,:) - u1(1,:));
    title('Autocorrelazione del rumore sulle accelerazioni X Veicolo 1');
    subplot(2,1,2)
    autocorr(u1_bar(2,:) - u1(2,:));
    title('Autocorrelazione del rumore sulle accelerazioni Y Veicolo 1');
    
    figure(5), clf, hold on;
    subplot(2,1,1)
    autocorr(x1Est(1,:));
    title('Autocorrelazione della stima della posizione X Veicolo 1');
    subplot(2,1,2)
    autocorr(x1Est(2,:));
    title('Autocorrelazione della stima della posizione Y Veicolo 1');
end

if PLOTHist == true
    % Istogramma degli errori di stima per il veicolo 1
    Error = x1Store(1,10:end) - x1Est(1,10:end);
    figure(6), clf, hold on;
    subplot(1,2,1)
    histogram(Error);
    title('Distribuzione degli errori di stima X - Veicolo 1');
    % Istogramma degli errori di stima per il veicolo 1
    Error = x1Store(2,10:end) - x1Est(2,10:end);
    subplot(1,2,2)
    histogram(Error);
    title('Distribuzione degli errori di stima Y - Veicolo 1');

    % disp('Varianza campionaria Veicolo 1:');
    % var(Error)
    % disp('Varianza stimata dal KF Veicolo 1:');
    % P1

    % Istogramma degli errori di stima per il veicolo 1
    Error = x2Store(1,10:end) - x2Est(1,10:end);
    figure(7), clf, hold on;
    subplot(1,2,1)
    histogram(Error);
    title('Distribuzione degli errori di stima X - Veicolo 1');
    % Istogramma degli errori di stima per il veicolo 1
    Error = x2Store(2,10:end) - x2Est(2,10:end);
    subplot(1,2,2)
    histogram(Error);
    title('Distribuzione degli errori di stima Y - Veicolo 1');

    % disp('Varianza campionaria Veicolo 2:');
    % var(Error)
    % disp('Varianza stimata dal KF Veicolo 2:');
    % P2
end
