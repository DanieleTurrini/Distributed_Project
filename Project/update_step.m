function [x,P] = update_step(x_p, measure, H, P_p, R)

    % Update step
    y = measure' - H * x_p';                % Innovation
    S = H * P_p * H' + R;                   % Covariance matrix of innovation
    W = P_p * H' / S;                       % Kalman Gain
    x = (x_p' + W * y)';                    % State Update
    P = (eye(size(P_p)) - W * H) * P_p;     % Covariance update