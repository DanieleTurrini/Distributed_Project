%% Consensus Example for a Linearly Increasing Temperature

% Parameters
N = 3;                % Number of sensors
T_sim = 50;           % Total simulation steps
dt = 1;               % Time step duration

% True temperature parameters: T(t) = a*t + b
a = 0.5;
b = 20;

% Consensus algorithm gains
gamma = 0.5;          % Weight for incorporating the sensor's own measurement
k = 0.1;              % Consensus gain for exchanging information with neighbors

% Noise parameters
noise_std = 0.1;      % Standard deviation of the sensor noise

% Initialize sensor estimates (starting around b with some random deviation)
x = b + randn(N,1)*5;

% Define neighbors (fully connected network in this example)
neighbors = { [2,3], [1,3], [1,2] };

% Record sensor estimates over time for plotting
x_history = zeros(N, T_sim);
time = zeros(1, T_sim);

for t = 1:T_sim
    % Compute the true temperature at this time step
    T_true = a*(t*dt) + b;
    
    % Store the current time for plotting
    time(t) = t*dt;
    
    % Each sensor takes a noisy measurement of the true temperature
    y = T_true + noise_std*randn(N,1);
    
    % Update estimates using consensus + measurement update
    x_new = zeros(N,1);
    for i = 1:N
        % Sum of differences between sensor i's estimate and its neighbors'
        consensus_sum = 0;
        for j = neighbors{i}
            consensus_sum = consensus_sum + (x(j) - x(i));
        end
        % Update rule: current estimate + consensus correction + measurement correction
        x_new(i) = x(i) + k*consensus_sum + gamma*(y(i) - x(i));
    end
    
    % Update sensor estimates
    x = x_new;
    
    % Record estimates for this time step
    x_history(:, t) = x;
end

% Plot the true temperature and sensor estimates over time
figure;
plot(time, a*time + b, 'k-', 'LineWidth', 2); hold on;
plot(time, x_history(1,:), 'r--', 'LineWidth', 1.5);
plot(time, x_history(2,:), 'b--', 'LineWidth', 1.5);
plot(time, x_history(3,:), 'g--', 'LineWidth', 1.5);
xlabel('Time');
ylabel('Temperature');
legend('True Temperature','Sensor 1','Sensor 2','Sensor 3','Location','NorthWest');
title('Consensus-Based Tracking of Linearly Increasing Temperature');
grid on;
