
%{
 function [plots] = plotSimulation_function(states, states_est, numUAV, dimgrid, x_fire1, y_fire1, sigma_fire1, x_fire2, y_fire2, sigma_fire2, x_water, y_water, sigma_water, numfig)
    % Funzione per visualizzare la simulazione dei droni e degli obiettivi
    
    figure(numfig); clf;
    subplot(1,2,1); % Plot 1: Droni reali
    colors = lines(numUAV);
    hold on;
    axis([0 dimgrid(1) 0 dimgrid(2) 0 dimgrid(3)]);
    xlabel('X Coordinate');
    ylabel('Y Coordinate');
    zlabel('Z Coordinate');
    title('Simulation');
    view(3);
    grid on;

    % Plot dei droni
    for i = 1:numUAV
        % Posizione attuale del drone
        plot3(states(i,1), states(i,2), states(i,3), 'o', 'Color', colors(i,:));
        % Disegna il drone come un unicycle
        drawUnicycle(states(i,1), states(i,2), states(i,4));
    end

    % Diagramma di Voronoi
    voronoi(states(:,1), states(:,2));

    % Plot dei fuochi
    plot3(x_fire1, y_fire1, 0, 'x', 'Color', 'r', 'MarkerSize', sigma_fire1);
    plot3(x_fire2, y_fire2, 0, 'x', 'Color', 'r', 'MarkerSize', sigma_fire2);

    % Plot dell'acqua
    r = sigma_water / 2; % Raggio del cerchio
    theta = linspace(0, 2*pi, 100); % Parametro angolare
    x_circle = r * cos(theta) + x_water;
    y_circle = r * sin(theta) + y_water;
    z_circle = zeros(size(theta));
    plot3(x_circle, y_circle, z_circle, 'b', 'LineWidth', 2);

    % drawnow; % Aggiorna la figura

    % Plot 2: Droni stimati
    subplot(1,2,2); % Plot 2: Droni stimati
    hold on;
    axis([0 dimgrid(1) 0 dimgrid(2) 0 dimgrid(3)]);
    xlabel('X Coordinate');
    ylabel('Y Coordinate');
    zlabel('Z Coordinate');
    title('Estimated Simulation');
    view(3);
    grid on;

    % Plot dei droni stimati
    for i = 1:numUAV
        % Posizione stimata del drone
        plot3(states_est(i,1), states_est(i,2), states_est(i,3), 'o', 'Color', colors(i,:));
        % Disegna il drone stimato come un unicycle
        drawUnicycle(states_est(i,1), states_est(i,2), states_est(i,4));
    end

    % Diagramma di Voronoi per i droni stimati
    voronoi(states_est(:,1), states_est(:,2));

    % Plot dei fuochi stimati
    plot3(x_fire1, y_fire1, 0, 'x', 'Color', 'r', 'MarkerSize', sigma_fire1);
    plot3(x_fire2, y_fire2, 0, 'x', 'Color', 'r', 'MarkerSize', sigma_fire2);

    % Plot dell'acqua stimata
    r = sigma_water / 2; % Raggio del cerchio
    theta = linspace(0, 2*pi, 100); % Parametro angolare
    x_circle = r * cos(theta) + x_water;
    y_circle = r * sin(theta) + y_water;
    z_circle = zeros(size(theta));
    plot3(x_circle, y_circle, z_circle, 'b', 'LineWidth', 2);


end 
%}
function plotSimulation_function(states, states_est, numUAV, dimgrid, x_fire1, y_fire1, sigma_fire1, x_fire2, y_fire2, sigma_fire2, x_water, y_water, sigma_water, numfig)
    % Funzione ottimizzata per la simulazione dei droni e degli obiettivi

    figure(numfig); clf;
    set(gcf, 'Position', [100, 100, 1200, 600]); % Imposta la dimensione della figura per adattarsi meglio allo schermo
    colors = lines(numUAV); % Genera una mappa di colori unica per ogni drone
    theta = linspace(0, 2 * pi, 100); % Parametro angolare precomputato
    r = sigma_water / 2;
    x_circle = r * cos(theta) + x_water;
    y_circle = r * sin(theta) + y_water;
    z_circle = zeros(size(theta));

    for subplot_idx = 1:2
        subplot(1, 2, subplot_idx);
        hold on;
        axis([0 dimgrid(1) 0 dimgrid(2) 0 dimgrid(3)]);
        xlabel('X Coordinate');
        ylabel('Y Coordinate');
        zlabel('Z Coordinate');

        if subplot_idx == 1
            title('Simulation');
            states_to_plot = states;
        else
            title('Estimated Simulation');
            states_to_plot = states_est;
        end

        view(3);
        grid on;

        % Plot dei droni con colori unici
        for i = 1:numUAV
            % Posizione del drone
            plot3(states_to_plot(i, 1), states_to_plot(i, 2), states_to_plot(i, 3), 'o', 'Color', colors(i, :), 'MarkerSize', 8);
            % Disegna il drone come un unicycle con il colore corrispondente
            drawUnicycle(states_to_plot(i, 1), states_to_plot(i, 2), states_to_plot(i, 4), colors(i, :));
        end

        % Diagramma di Voronoi
        voronoi(states_to_plot(:, 1), states_to_plot(:, 2));

        % Plot dei fuochi
        plot3(x_fire1, y_fire1, 0, 'x', 'Color', 'r', 'MarkerSize', sigma_fire1);
        plot3(x_fire2, y_fire2, 0, 'x', 'Color', 'r', 'MarkerSize', sigma_fire2);

        % Plot dell'acqua
        plot3(x_circle, y_circle, z_circle, 'b', 'LineWidth', 2);
    end
end