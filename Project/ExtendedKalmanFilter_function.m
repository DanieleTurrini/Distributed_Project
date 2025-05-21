%EXTENDEDKALMANFILTER_FUNCTION Performs one iteration of the Extended Kalman Filter (EKF).
%   [x, P] = ExtendedKalmanFilter_function(states_est, measure, control, A, G, fun, Q, h, H, R, P, count, meas_freq_GPS, meas_freq_ultr, meas_freq_gyr, dt)
%   predicts the next state and updates the state estimate and covariance
%   using probabilistic measurements from GPS, ultrasonic, and gyroscope sensors.
%   The function handles prediction and selective measurement updates based on
%   sensor availability and measurement probabilities.

function [x, P] = ExtendedKalmanFilter_function(states_est, measure, control, A, G, fun, Q, h, H, R, P, count, meas_freq_GPS, meas_freq_ultr, meas_freq_gyr, dt)
    
    %% Measure Probability
    probGPS = 0.9;
    probUltr = 0.8;
    probGyr = 0.7;

    %% Prediction step
    x = fun(states_est, control, dt);      % State prediction
    A_k = A(control, states_est(4), dt);    
    G_k = G(states_est(4), dt);
    P = A_k * P * A_k' + G_k * Q * G_k';   % Covariance prediction
    
    %% Update step
    z_ = h(x,states_est(4),dt , [0,0,0,0]); % Measurement prediction    
    % GPS
    if mod(count, meas_freq_GPS) == 0 && rand(1) <= probGPS

        [x,P] = update_step(x, measure(1:2), H(1:2,:), P, R(1:2,1:2),z_(1:2));
    end

    % Ultrasonic
    if mod(count, meas_freq_ultr) == 0 && rand(1) <= probUltr

        [x,P] = update_step(x, measure(3), H(3,:), P, R(3,3),z_(3));
    end
    % Gyroscope
    if mod(count, meas_freq_gyr) == 0 && rand(1) <= probGyr

        [x,P] = update_step(x, measure(4), H(4,:), P, R(4,4), z_(4));

    end
    %disp(P(4,4));
    
end
