function [x, P] = ExtendedKalmanFilter_function(states_est, measure, control, A, G, fun, Q, H, R, P, count, meas_freq_GPS, meas_freq_ultr, meas_freq_gyr, dt)
    
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
    % GPS
    if mod(count, meas_freq_GPS) == 0 && rand(1) <= probGPS

        [x,P] = update_step(x, measure(1:2), H(1:2,:), P, R(1:2,1:2));
    end
    % Ultrasonic
    if mod(count, meas_freq_ultr) == 0 && rand(1) <= probUltr

        [x,P] = update_step(x, measure(3), H(3,:), P, R(3,3));
    end
    % Gyroscope
    if mod(count, meas_freq_gyr) == 0 && rand(1) <= probGyr

        [x,P] = update_step(x, measure(4), H(4,:), P, R(4,4));

    end
    
end
