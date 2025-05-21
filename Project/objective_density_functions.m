% OBJECTIVE_DENSITY_FUNCTIONS Computes and optionally plots Gaussian density functions for fire sources, water source, and UAV landing zone.
%   returns the combined fire density (G_fires) and water source density (G_water) over a grid, and can visualize these along with the UAV landing density.

function [G_fires, G_water] = objective_density_functions(dimgrid,pos_fire1,pos_fire2,pos_water,sigma_fire1,sigma_fire2,sigma_water,t,initialUAV_pos, PLOT_DENSITY_FUNCTIONS)
    

    % Creation of the grid of points
    [x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));
    % Drone to plot
    i = 1;

    posFireMov1 = pos_fire1(t);
    posFireMov2 = pos_fire2(t);

    % Calculation of the Gaussian distribution
    G_fires = exp(-(((x_m - posFireMov1(1)).^2) / (2 * sigma_fire1^2) + ((y_m - posFireMov1(2)).^2) / (2 * sigma_fire1^2))) + ...
            exp(-(((x_m - posFireMov2(1)).^2) / (2 * sigma_fire2^2) + ((y_m - posFireMov2(2)).^2) / (2 * sigma_fire2^2)));

    G_water = exp(-(((x_m - pos_water(1)).^2) / (2 * sigma_water^2) + ((y_m - pos_water(2)).^2) / (2 * sigma_water^2)));

    % Landing Distribution

    sigma = 20; 
    G_landing = exp( -(((x_m - initialUAV_pos(i,1)).^2 + (y_m - initialUAV_pos(i,2)).^2) / (2 * sigma^2)) );

    if PLOT_DENSITY_FUNCTIONS
        figure('Units','normalized','Position',[0.1, 0.2, 0.8, 0.6]); 

        % First subplot for the fire density plot
        subplot(1, 3, 1);
        surf(x_m, y_m, G_fires,'FaceAlpha', 0.8);
        shading interp; % Makes the surface smoother
        colormap jet;
        xlabel('X');
        ylabel('Y');
        zlabel('Density');
        title('Fires density function');
        hold on;

        zLabelHeight = max(G_fires(:)) * 0.9;
        text(posFireMov1(1), posFireMov1(2), zLabelHeight, 'Fire 1', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');
        text(posFireMov2(1), posFireMov2(2), zLabelHeight, 'Fire 2', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');
        hold off;
        
        % Second subplot for the water source density plot
        subplot(1, 3, 2);
        surf(x_m, y_m, G_water,'FaceAlpha', 0.8);
        shading interp;
        xlabel('X');
        ylabel('Y');
        zlabel('Density');
        title('Water Source Density Function');
        hold on;

        zLabelHeight = max(G_water(:)) * 0.9;
        text(pos_water(1), pos_water(2), zLabelHeight, 'Water Source', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');
        hold off;

        % Third subplot for the landing density plot
        subplot(1, 3, 3);
        surf(x_m, y_m, G_landing,'FaceAlpha', 0.8);
        shading interp;
        xlabel('X');
        ylabel('Y');
        zlabel('Density');
        title(sprintf('Landing Density Function for UAV %d', i));
        hold on;

        zLabelHeight = max(G_landing(:)) * 0.9;
        text(initialUAV_pos(i,1), initialUAV_pos(i,2), zLabelHeight, 'Landing Zone', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');
        hold off;

        % Add a shared colorbar
        cb = colorbar('Position', [0.92, 0.1, 0.02, 0.8]); % Adjust position as needed
        ylabel(cb, 'Density');
        
        drawnow;
    end

end