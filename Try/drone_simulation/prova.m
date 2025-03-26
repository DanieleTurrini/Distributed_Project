% Parametri di simulazione
dt = 0.01;       % intervallo di tempo
T = 20;          % tempo totale di simulazione [s]
time = 0:dt:T;
nSteps = length(time);

% Parametri del percorso circolare per il Punto 2
R = 5;           % raggio della circonferenza
omega = 1;     % velocità angolare [rad/s]

% Preallocazione degli array per le traiettorie
pos1 = zeros(nSteps, 2);  % posizione del Punto 1 (seguace)
vel1 = zeros(nSteps, 2);  % velocità del Punto 1
acc1 = zeros(nSteps, 2);  % accelerazione comandata (output PID)

pos2 = zeros(nSteps, 2);  % posizione del Punto 2 (target)

% Guadagni PID (da tarare in base alla simulazione)
Kp = 10;
Ki = 0.5;
Kd = 5;

error_integral = [0, 0];  % integrale dell'errore
prev_error = [0, 0];      % errore precedente

% Condizioni iniziali: Punto 1 in (0,0)
pos1(1,:) = [0, 0];
vel1(1,:) = [0, 0];

figure(1)
xlabel('X');
ylabel('Y');
title('Simulazione');
legend('Punto 2 (target)', 'Punto 1 (seguace)');
axis([-6 6 -6 6]);
hold on;


% Ciclo di simulazione
for k = 1:nSteps-1
    % Calcolo della posizione del Punto 2 lungo la circonferenza
    pos2(k,:) = [R*cos(omega*time(k)), R*sin(omega*time(k))];
    
    % Calcolo dell'errore di posizione (vettoriale)
    error = pos2(k,:) - pos1(k,:);
    
    % Aggiornamento dell'integrale dell'errore
    error_integral = error_integral + error * dt;
    
    % Calcolo della derivata dell'errore
    error_derivative = (error - prev_error) / dt;
    
    % Controllo PID per ottenere l'accelerazione comando
    a_command = Kp * error + Ki * error_integral + Kd * error_derivative;
    acc1(k,:) = a_command;
    
    % Integrazione delle accelerazioni per ottenere velocità e posizione (Euler)
    vel1(k+1,:) = vel1(k,:) + a_command * dt;
    pos1(k+1,:) = pos1(k,:) + vel1(k,:) * dt;
    
    % Aggiornamento dell'errore precedente
    prev_error = error;
    
    cla;
    plot(pos2(k,1), pos2(k,2),'ro'); 
    plot(pos1(k,1), pos1(k,2),'bo');
    drawnow;  % aggiorna la figura

end
% Calcolo dell'ultima posizione del Punto 2
pos2(end,:) = [R*cos(omega*time(end)), R*sin(omega*time(end))];

% Visualizzazione della traiettoria
figure(2);
plot(pos2(:,1), pos2(:,2), 'r--', 'LineWidth', 2); hold on;
plot(pos1(:,1), pos1(:,2), 'b', 'LineWidth', 2);
legend('Punto 2 (target)', 'Punto 1 (seguace)');
xlabel('X');
ylabel('Y');
title('Controllo PID: Punto 1 segue Punto 2 lungo una circonferenza');
grid on;
axis equal;
