function [xEst, P] = kalmanFilter_SingleDrone_Function(xEst, P, u_bar, sigma_u, sigma_gps, measure, ProbGPS, dt, MapDim)
    % Prediction step
    A = eye(2); % Matrice di transizione di stato (identità per il modello semplice)
    Q = sigma_u^2 * eye(2);  % Covarianza del rumore di processo

    xPred = A * xEst(:) + u_bar(:) * dt;  % Stato predetto
    Ppred = A * P * A' + Q;  % Covarianza dello stato predetto

    % Le coordinate dei droni non possono esse fuori dalla mappa 
    xPred(1) = max(1, min(MapDim(1), xPred(1)));
    xPred(2) = max(1, min(MapDim(2), xPred(2)));
    
    % Update step
    H = eye(2);  % Matrice di osservazione (identità per posizione diretta)
    R = sigma_gps^2 * eye(2);  % Covarianza del rumore di misura

    if rand < ProbGPS  % Con probabilità ProbGPS, aggiorniamo con la misura
        S = H * Ppred * H' + R;  % Innovazione della covarianza
        W = Ppred * H' / S;  % Guadagno di Kalman
        xEst = xPred + W * (measure(:) - H * xPred);  % Stato aggiornato
        P = (eye(2) - W * H) * Ppred;  % Covarianza aggiornata
    else
        xEst = xPred;
        P = Ppred;
    end
end