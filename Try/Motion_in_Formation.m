% Parameters
N = 5;                    % Number of robots
dt = 0.01;                % Time step
T = 20;                   % Total simulation time
time = 0:dt:T;
alpha = 4.0;              % Proportional gain for destination reaching
gamma = 5;              % Damping coefficient
k = 0.3;                  % Coupling gain for formation keeping

% Desired destination for each robot (for simplicity, same destination)
x_d = 10 * ones(N,1);
y_d = 10 * ones(N,1);

% Initial positions and velocities (randomly distributed)
x = rand(N,1) * 5;
y = rand(N,1) * 5;
vx = zeros(N,1);
vy = zeros(N,1);

% Define neighbors (adjacency matrix) - simple example: ring topology
G = zeros(N);
for i = 1:N
    % Each robot has neighbors i-1 and i+1 (with wrapping)
    G(i, mod(i-2, N) + 1) = 1;
    G(i, mod(i, N) + 1) = 1;
end

% Simulation
figure;
hold on;
for t = time
    % Compute acceleration commands for each robot
    ax = zeros(N,1);
    ay = zeros(N,1);
    
    for i = 1:N
        % Destination reaching component
        ax_des = -alpha * (x(i) - x_d(i)) - gamma * vx(i);
        ay_des = -alpha * (y(i) - y_d(i)) - gamma * vy(i);
        
        % Formation keeping (consensus) component
        consensus_x = 0;
        consensus_y = 0;
        for j = 1:N
            if G(i,j) == 1
                % Position difference (you may also include velocity differences)
                consensus_x = consensus_x - k * ( (x(i)-x_d(i)) - (x(j)-x_d(j)) ) ...
                              + gamma * k * (vx(i) - vx(j));
                consensus_y = consensus_y - k * ( (y(i)-y_d(i)) - (y(j)-y_d(j)) ) ...
                              + gamma * k * (vy(i) - vy(j));
            end
        end
        
        % Total acceleration
        ax(i) = ax_des + consensus_x;
        ay(i) = ay_des + consensus_y;
    end
    
    % Update velocities and positions using Euler integration
    vx = vx + ax * dt;
    vy = vy + ay * dt;
    x = x + vx * dt;
    y = y + vy * dt;
    
    % Plot positions of the robots
    clf;
    plot(x, y, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    hold on;
    plot(x_d, y_d, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    xlim([0 12]);
    ylim([0 12]);
    title(sprintf('Time = %.2f seconds', t));
    xlabel('X Position');
    ylabel('Y Position');
    drawnow;
end
