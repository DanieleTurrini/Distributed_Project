clear; close all; clc;

% Definizione del passo di tempo
dT = 1/30; % Tempo di campionamento (30 Hz)

% Funzione di transizione di stato (modello non lineare del sistema)
fun = @(x, y, theta, vel, omega) [x + vel * cos(theta) * dT;  
                                  y + vel * sin(theta) * dT;  
                                  theta + omega * dT];

% Jacobiano del modello di stato (matrice A del EKF)
A = @(x, y, theta, vel) [1, 0, -vel * sin(theta) * dT;
                          0, 1, vel * cos(theta) * dT;
                          0, 0, 1];

% Matrice di propagazione del rumore di processo (G)
G = @(theta) [cos(theta) * dT, 0;
              sin(theta) * dT, 0;
              0, dT];

% Covarianza del rumore di processo (incertezza sulla dinamica)
std_u = [0.1, 0.1]; % Incertezza sulla velocità e sulla velocità angolare
Q = diag(std_u.^2); 

% Stato iniziale [posizione_x, posizione_y, orientazione]
state = [4; 2; 0];

% Parametri della misura (GPS per posizione, giroscopio per orientazione)
std_gps = 0.1; % Deviazione standard del GPS
std_gyro = 0.05; % Deviazione standard del giroscopio
R = diag([std_gps^2, std_gps^2, std_gyro^2]); % Covarianza del rumore di misura

% Misura iniziale con rumore
z = state + [std_gps * randn(2, 1); std_gyro * randn(1, 1)];

% Matrice di osservazione (H)
H = eye(3);

% Matrice di covarianza iniziale della stima
P = eye(3);

% Stima iniziale con metodo dei minimi quadrati
x_est = state; 

% Numero di iterazioni
Ttot = 200;

% Variabili per grafico
x_real = zeros(Ttot, 3);
x_estimato = zeros(Ttot, 3);
trace_P = zeros(Ttot, 1);

% Inizializzazione grafico
figure;
hold on;
real_trajectory = plot(NaN, NaN, 'bo-', 'MarkerSize', 5, 'DisplayName', 'Traiettoria Reale');
estimated_trajectory = plot(NaN, NaN, 'g+', 'MarkerSize', 10, 'DisplayName', 'Stima EKF');
title('Filtro di Kalman Esteso - Stima della Traiettoria');
xlabel('x');
ylabel('y');
grid on;
legend;

% Parametri di movimento
vel = 1; % Velocità costante
omega = 0.4; % Velocità angolare costante

% Loop EKF
for k = 1:Ttot
    % Simulazione del movimento reale del robot
    state = fun(state(1), state(2), state(3), vel, omega);
    
    % Misura GPS e giroscopio con rumore
    z = state + [std_gps * randn(2, 1); std_gyro * randn(1, 1)];

    % EKF - Fase di Predizione
    x_pred = fun(x_est(1), x_est(2), x_est(3), vel, omega);
    A_k = A(x_est(1), x_est(2), x_est(3), vel);
    G_k = G(x_est(3));
    P_pred = A_k * P * A_k' + G_k * Q * G_k';

    % EKF - Fase di Aggiornamento
    K = P_pred * H' / (H * P_pred * H' + R);
    x_est = x_pred + K * (z - H * x_pred);
    P = (eye(3) - K * H) * P_pred;

    % Salvataggio dati per il grafico
    x_real(k, :) = state';
    x_estimato(k, :) = x_est';

    % Aggiornamento grafico
    set(real_trajectory, 'XData', x_real(1:k, 1), 'YData', x_real(1:k, 2));
    set(estimated_trajectory, 'XData', x_estimato(1:k, 1), 'YData', x_estimato(1:k, 2));
    drawnow;
    pause(0.05);
    

    % Traccia della Matrice di Covarianza
     trace_P(k) = trace(P);
end

% Grafico della Traccia della Covarianza
figure;
plot(trace_P);
title('Traccia della Matrice di Covarianza');
xlabel('Passo Temporale');
ylabel('Traccia della Matrice di Covarianza');

