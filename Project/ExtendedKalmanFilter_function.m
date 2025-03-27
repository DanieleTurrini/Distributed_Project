function [states_est, P] = ExtendedKalmanFilter_function(states_est, measure, control, A, B, J_A, G,  Q, H, R, P, dt)
    % Prediction step
    x_pred = A * states_est + B * control;
    J_A = J_A(control, states_est(4));
    P_pred = J_A * P * J_A' + G * Q * G';
    
    % Update step
    W = P_pred * H' / (H * P_pred * H' + R);
    states_est = x_pred + W * (z - H * x_pred);
    P = (eye(size(P)) - W * H) * P_pred;
end