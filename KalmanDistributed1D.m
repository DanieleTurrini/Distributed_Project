clear all;
close all;
clc;

%% Inizializzazione del sistema per i due veicoli

% Parametri generali
Dt = 0.1; % Passo di campionamento (100 ms)
t = 0:Dt:100; % Intervallo di simulazione

%% Veicolo 1
x1 = rand(1); % Posizione iniziale casuale
u1 = 1 + sin(t); % Comando di controllo (accelerazione variabile nel tempo)

%% Veicolo 2
x2 = 2 + rand(1); % Posizione iniziale casuale
u2 = 1 + cos(t); % Comando di controllo per il veicolo 2

% Effetto dell'attrito
% Parametri che descrivono la dinamica del veicolo
 a = -0.1;
 b = Dt * 1.2;

%% Simulazione della dinamica dei veicoli
x1Store = zeros(1, length(t));
x1Store(1) = x1;
x2Store = zeros(1, length(t));
x2Store(1) = x2;

for i = 1:length(t)-1
    % Aggiornamento della posizione del veicolo 1
    x1Store(i+1) = a * x1Store(i) + b * u1(i);
    % Aggiornamento della posizione del veicolo 2
    x2Store(i+1) = a * x2Store(i) + b * u2(i);
end

%% Introduzione delle incertezze nelle misurazioni

% Errore nelle misure di accelerazione
sigma_u1 = 0.1;
u1_bar = u1 + randn(1, length(u1)) * sigma_u1;
sigma_u2 = 0.05;
u2_bar = u2 + randn(1, length(u2)) * sigma_u2;

% Errore nelle misure GPS
sigma_gps1 = 0.3;
x1GPS = x1Store + randn(1, length(x1Store)) * sigma_gps1;
ProbGPS1 = 0.05; % Probabilità che il GPS sia disponibile

sigma_gps2 = 0.2;
x2GPS = x2Store + randn(1, length(x2Store)) * sigma_gps2;
ProbGPS2 = 0.9;

% Errore nelle misure Radar
sigma_radar = 0.1;
ProbRadar = 0.8;

%% Inizializzazione del Filtro di Kalman

% Veicolo 1
x1Est = zeros(1, length(t));
P1 = 100;
P1Store = zeros(1, length(t));
P1PredStore = zeros(1, length(t));

% Veicolo 2
x2Est = zeros(1, length(t));
P2 = 100;
P2Store = zeros(1, length(t));
P2PredStore = zeros(1, length(t));

for i = 1:length(t)-1
    %% Filtro di Kalman per il veicolo 1
    
    % Predizione
    x1EstPred = a * x1Est(i) + b * u1_bar(i);
    P1pred = a * P1 * a' + b * sigma_u1^2 * b';
    
    % Aggiornamento basato su GPS
    if rand(1) <= ProbGPS1
        H = 1;
        InnCov = H * P1pred * H' + sigma_gps1^2;
        W = P1pred * H' / InnCov;
        x1Est(i+1) = x1EstPred + W * (x1GPS(i+1) - H * x1EstPred);
        P1 = (1 - W * H) * P1pred;
    else
        x1Est(i+1) = x1EstPred;
        P1 = P1pred;
    end
    
    P1Store(i+1) = P1;
    P1PredStore(i+1) = P1pred;
    
    %% Filtro di Kalman per il veicolo 2
    
    % Predizione
    x2EstPred = a * x2Est(i) + b * u2_bar(i);
    P2pred = a * P2 * a' + b * sigma_u2^2 * b';
    
    % Aggiornamento basato su GPS
    if rand(1) <= ProbGPS2
        H = 1;
        InnCov = H * P2pred * H' + sigma_gps2^2;
        W = P2pred * H' / InnCov;
        x2Est(i+1) = x2EstPred + W * (x2GPS(i+1) - H * x2EstPred);
        P2 = (1 - W * H) * P2pred;
    else
        x2Est(i+1) = x2EstPred;
        P2 = P2pred;
    end
    
    P2Store(i+1) = P2;
    P2PredStore(i+1) = P2pred;
    
end

%% Visualizzazione dei risultati

% Confronto tra posizioni reali e stimate
figure(1), clf, hold on;
plot(t, x1Store, 'b');
plot(t, x1Est, 'r');
legend('x1 Reale', 'x1 Stimato');
xlabel('Tempo [s]');
ylabel('Posizione [m]');

title('Posizione reale vs stimata per il veicolo 1');

% Andamento della covarianza
figure(2), clf, hold on;
plot(t, P1Store, 'b');
plot(t, P1PredStore, 'r');
plot(t, P2Store, 'k--');
plot(t, P2PredStore, 'g--');
legend('P1', 'P1 Predetto', 'P2', 'P2 Predetto');
xlabel('Tempo [s]');
ylabel('Varianza');
set(gca, 'YScale', 'log');
title('Andamento della covarianza di errore');

% Istogramma degli errori di stima per il veicolo 1
Error = x1Store(10:end) - x1Est(10:end);
figure(3), clf, hold on;
histogram(Error);
title('Distribuzione degli errori di stima - Veicolo 1');
disp('Varianza campionaria Veicolo 1:');
var(Error)
disp('Varianza stimata dal KF Veicolo 1:');
P1

% Autocorrelazione delle misure
figure(4), clf, hold on;
autocorr(u1_bar - u1);
title('Autocorrelazione del rumore sulle accelerazioni');

figure(5), clf, hold on;
autocorr(x1Est);
title('Autocorrelazione della stima della posizione');

% Istogramma degli errori di stima per il veicolo 2
Error = x2Store(10:end) - x2Est(10:end);
figure(6), clf, hold on;
histogram(Error);
title('Distribuzione degli errori di stima - Veicolo 2');
disp('Varianza campionaria Veicolo 2:');
var(Error)
disp('Varianza stimata dal KF Veicolo 2:');
P2
