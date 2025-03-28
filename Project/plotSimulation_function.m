function [plots] = plotSimulation_function(states, numUAV, dimgrid, x_fire1, y_fire1, sigma_fire1, x_fire2, y_fire2, sigma_fire2, x_water, y_water, sigma_water, numfig)
    % Funzione per visualizzare la simulazione dei droni e degli obiettivi
    
    figure(numfig); clf;
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
end