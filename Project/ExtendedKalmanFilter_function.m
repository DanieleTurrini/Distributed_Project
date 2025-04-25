function [x, P] = ExtendedKalmanFilter_function(states_est, measure, control, A, G, fun, Q, H, R, P, count, meas_freq_GPS, meas_freq_ultr, meas_freq_gyr, dt)
    
    % Measure Probability
    probGPS = 0.9;
    probUltr = 0.8;
    probGyr = 0.7;

    %% Prediction step
    x = fun(states_est, control, dt);      % State prediction
    A_k = A(control, states_est(4), dt);    
    G_k = G(states_est(4), dt);
    P = A_k * P * A_k' + G_k * Q * G_k';   % Covariance prediction

    %% Update step
    if mod(count, meas_freq_GPS) == 0 && rand(1) <= probGPS

        H = H(1:2,:);
        R = R(1:2,1:2);
        measure = measure(1:2);
        [x,P] = update_step(x, measure, H, P, R);

    elseif mod(count, meas_freq_ultr) == 0 && rand(1) <= probUltr

        [x,P] = update_step(x, measure(4), H(4,:), P, R(4,4));

    elseif mod(count, meas_freq_gyr) == 0 && rand(1) <= probGyr

        H = H(4,:);
        R = R(4,4);
        measure = measure(4);
        [x,P] = update_step(x, measure, H, P, R);

    end
    
end