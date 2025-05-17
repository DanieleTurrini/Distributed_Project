function [x,P] = update_step(x_p, measure, H, P_p, R,z_)

    % Update step
    y = measure' - z_';                % Innovation
    S = H * P_p * H' + R;                   % Covariance matrix of innovation
    W = P_p * H' / S;                       % Kalman Gain
    x = (x_p' + W * y)';                    % State Update
    P = (eye(size(P_p)) - W * H) * P_p;     % Covariance update