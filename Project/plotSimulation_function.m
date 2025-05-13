function plotSimulation_function(states, states_est, centroids_est, numUAV, dimgrid, pos_est_fire1, x_fire1, y_fire1, sigma_est_fire1, curr_fire1_sig, x_fire2, y_fire2, sigma_fire2, x_water, y_water, sigma_water, Xf, Yf, Zf, dim_UAV, numfig)
    % Funzione ottimizzata per la simulazione dei droni e degli obiettivi

    figure(numfig); clf;
    subplot(1,2,1);
    set(gcf, 'Position', [0, 100, 1400, 600]); % Imposta la dimensione della figura per adattarsi meglio allo schermo
    theta = linspace(0, 2 * pi, 100); % Parametro angolare precomputato
    r =  45;
    x_circle = r * cos(theta) + x_water;
    y_circle = r * sin(theta) + y_water;
    z_circle = ones(size(theta)) * 5;

    axis([0 dimgrid(1) 0 dimgrid(2) 0 dimgrid(3)]);
    xlabel('X Coordinate');
    ylabel('Y Coordinate');
    zlabel('Z Coordinate');

    grid on;
    hold on;
    
    surf(Xf, Yf, Zf, 'FaceColor', [0.4660 0.6740 0.1880], 'FaceAlpha', 0.9, 'EdgeColor', 'none');

    contour3(Xf, Yf, Zf, 20, 'k');  % '20' = numero di livelli, 'k' = colore nero

    
    for i = 1:numUAV

        % Draw the UAV pose
        drawUAV(states(i, 1), states(i, 2), states(i, 3), states(i, 4), dim_UAV,'k');
        
        % Plot estimated positions of fire
        plot3(pos_est_fire1(i,1), pos_est_fire1(i,2), enviroment_surface(pos_est_fire1(i,1), pos_est_fire1(i,2), 1), 'x','MarkerSize', sigma_est_fire1(i,1));
        
    end

    % Plot dei fuochi
    plot3(x_fire1, y_fire1, enviroment_surface(x_fire1, y_fire1, 1), 'x', 'Color', 'r', 'MarkerSize', curr_fire1_sig, 'LineWidth', 2);
    plot3(x_fire2, y_fire2, enviroment_surface(x_fire2, y_fire2, 1), 'x', 'Color', 'r', 'MarkerSize', sigma_fire2, 'LineWidth', 2);

    % Plot dell'acqua
    plot3(x_circle, y_circle, z_circle, 'b', 'LineWidth', 2);

    hold off;
    view(3);

    subplot(1,2,2);

    axis([0 dimgrid(1) 0 dimgrid(2)]);
    xlabel('X Coordinate');
    ylabel('Y Coordinate');

    hold on;
    plot(x_fire1,y_fire1,'x','Color', 'r', 'MarkerSize', curr_fire1_sig)
    plot(x_fire2,y_fire2,'x','Color', 'r', 'MarkerSize', sigma_fire2)
    plot(x_water,y_water,'o','Color', 'b', 'MarkerSize', sigma_water)

    for i = 1:numUAV
        % Draw the UAV pose
        drawUAV2D(states(i, 1), states(i, 2), states(i, 4), dim_UAV,'k');
        drawUAV2D(states_est(i, 1), states_est(i, 2), states_est(i, 4), dim_UAV,'g');

        plot(centroids_est(i,1), centroids_est(i,2), 'x', 'Color', 'g');
        plot(pos_est_fire1(i,1), pos_est_fire1(i,2), 'x','MarkerSize', sigma_est_fire1(i,1));
    end

    [vx_es, vy_es] = voronoi(states_est(:,1), states_est(:,2)); 
    plot(vx_es, vy_es, 'g-');
    hold off;
    view(2);
end
