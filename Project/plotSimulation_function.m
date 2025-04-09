function plotSimulation_function(states, states_est, centroids_est, numUAV, dimgrid, pos_est_fire1, x_fire1, y_fire1, sigma_est_fire1, curr_fire1_sig, x_fire2, y_fire2, sigma_fire2, x_water, y_water, sigma_water, Xf, Yf, Zf, dim_UAV, numfig)
    % Funzione ottimizzata per la simulazione dei droni e degli obiettivi

    figure(numfig); clf;
    set(gcf, 'Position', [50, 50, 1600, 800]); % Imposta la dimensione della figura per adattarsi meglio allo schermo
    theta = linspace(0, 2 * pi, 100); % Parametro angolare precomputato
    r = sigma_water / 2;
    x_circle = r * cos(theta) + x_water;
    y_circle = r * sin(theta) + y_water;
    z_circle = zeros(size(theta));

    axis([0 dimgrid(1) 0 dimgrid(2) 0 dimgrid(3)]);
    xlabel('X Coordinate');
    ylabel('Y Coordinate');
    zlabel('Z Coordinate');

    grid on;
    hold on;
    
    % surf(Xf, Yf, Zf, 'FaceColor', [0.4660 0.6740 0.1880], 'FaceAlpha', 0.5, 'EdgeColor', 'none');

    % Plot dei droni con colori unici
    for i = 1:numUAV
        % Draw the UAV pose
        drawUAV(states(i, 1), states(i, 2), states(i, 3), states(i, 4), dim_UAV,'k');
        drawUAV(states_est(i, 1), states_est(i, 2), states_est(i, 3), states_est(i, 4), dim_UAV,'g');
    end

    % Diagramma di Voronoi
    % voronoi(states(:, 1), states(:, 2));

    % [vx_es, vy_es] = voronoi(states_est(:,1), states_est(:,2)); 
    % plot(vx_es, vy_es, 'g-');

    % Plot dei fuochi
    % plot3(x_fire1, y_fire1, flight_surface(x_fire1, y_fire1, 1), 'x', 'Color', 'r', 'MarkerSize', curr_fire1_sig);
    % plot3(x_fire2, y_fire2, flight_surface(x_fire2, y_fire2, 1), 'x', 'Color', 'r', 'MarkerSize', sigma_fire2);

    plot3(x_fire1, y_fire1, 0, 'x', 'Color', 'r', 'MarkerSize', curr_fire1_sig);
    plot3(x_fire2, y_fire2, 0, 'x', 'Color', 'r', 'MarkerSize', sigma_fire2);

    % Plot dei centroidi
    for i = 1:numUAV
        % plot3(centroids(i,1), centroids(i,2), 0, 'x', 'Color', 'b');
        plot3(centroids_est(i,1), centroids_est(i,2), 0, 'x', 'Color', 'g');
        % plot3(pos_est_fire1(i,1), pos_est_fire1(i,2), flight_surface(pos_est_fire1(i,1), pos_est_fire1(i,2), 1), 'x','MarkerSize', sigma_est_fire1(i,1));
        plot3(pos_est_fire1(i,1), pos_est_fire1(i,2), 0, 'x','MarkerSize', sigma_est_fire1(i,1));

    end

    % Plot dell'acqua
    plot3(x_circle, y_circle, z_circle, 'b', 'LineWidth', 2);

    hold off;
    view(3);
end
