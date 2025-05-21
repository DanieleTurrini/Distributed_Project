%UPDATE_STEP Performs the update step of the Kalman filter.
%   [x, P] = UPDATE_STEP(x_p, measure, H, P_p, R, z_) updates the predicted
%   state estimate x_p and covariance P_p using the measurement 'measure',
%   measurement matrix H, measurement noise covariance R, and predicted
%   measurement z_. Returns the updated state x and covariance P.

function [x,P] = update_step(x_p, measure, H, P_p, R,z_)

    % Update step
    y = measure' - z_';                % Innovation
    S = H * P_p * H' + R;                   % Covariance matrix of innovation
    W = P_p * H' / S;                       % Kalman Gain
    x = (x_p' + W * y)';                    % State Update
    P = (eye(size(P_p)) - W * H) * P_p;     % Covariance update