function [states_est, P] = ExtendedKalmanFilter_function(states_est, measure, control, A, G, fun, Q, H, R, P, dt)
    
    % Prediction step
    x_pred = fun(states_est, control, dt);      % State prediction
    A_k = A(control, states_est(4), dt);    
    G_k = G(states_est(4), dt);
    P_pred = A_k * P * A_k' + G_k * Q * G_k';   % Covariance prediction
    
    % Update step
    y = measure' - H * x_pred';                 % Innovation
    S = H * P_pred * H';                        % Covariance matrix of innovation
    W = P_pred * H' / (S + R);                  % Kalman Gain
    states_est = (x_pred' + W * y)';            % State Update
    P = (eye(size(P)) - W * H) * P_pred;        % Covariance update
    
end