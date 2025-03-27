function [states_est, P] = ExtendedKalmanFilter_function(states_est, measure, control, J_A, G, fun,  Q, H, R, P, dt)
    % Prediction step
    x_pred = fun(states_est, control, dt);
    J_A_k = J_A(control, states_est(4), dt);
    G_k = G(states_est(4), dt);
    P_pred = J_A_k * P * J_A_k' + G_k * Q * G_k';
    
    % Update step
    W = P_pred * H' / (H * P_pred * H' + R);
    states_est = (x_pred' + W * (measure' - H * x_pred'))';
    P = (eye(size(P)) - W * H) * P_pred;
end