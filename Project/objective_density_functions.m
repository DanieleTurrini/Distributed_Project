function [G_fires, G_water] = objective_density_functions(dimgrid,pos_fire1,pos_fire2,pos_water,sigma_fire1,sigma_fire2,sigma_water,t, PLOT_DENSITY_FUNCTIONS)
    

% Creazione della griglia di punti
[x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));

posFireMov1 = pos_fire1(t);

% Calcolo della distribuzione gaussiana
G_fires = exp(-(((x_m - posFireMov1(1)).^2) / (2 * sigma_fire1^2) + ((y_m - posFireMov1(2)).^2) / (2 * sigma_fire1^2))) + ...
          exp(-(((x_m - pos_fire2(1)).^2) / (2 * sigma_fire2^2) + ((y_m - pos_fire2(2)).^2) / (2 * sigma_fire2^2)));
G_water = exp(-(((x_m - pos_water(1)).^2) / (2 * sigma_water^2) + ((y_m - pos_water(2)).^2) / (2 * sigma_water^2)));

if PLOT_DENSITY_FUNCTIONS
    figure('Units','normalized','Position',[0.1, 0.2, 0.8, 0.6]); 

    % First subplot for the fire density plot
    subplot(1, 2, 1);
    surf(x_m, y_m, G_fires);
    shading interp; % Makes the surface smoother
    colormap jet;
    colorbar;
    xlabel('X');
    ylabel('Y');
    zlabel('Density');
    title('Fires density function');
    hold on;

    zLabelHeight = max(G_fires(:)) * 0.9;
    
    text(posFireMov1(1), posFireMov1(2), zLabelHeight, 'Fire 1', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');
    text(pos_fire2(1), pos_fire2(2), zLabelHeight, 'Fire 2', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');
    
    hold off;
    
    % Second subplot for the water source density plot
    subplot(1, 2, 2);
    surf(x_m, y_m, G_water);
    shading interp;
    colormap jet;
    colorbar;
    xlabel('X');
    ylabel('Y');
    zlabel('Density');
    title('Water Source Density Function');
    hold on;

    zLabelHeight = max(G_water(:)) * 0.9;
    
    text(pos_water(1), pos_water(2), zLabelHeight, 'Water Source', 'Color', 'k', 'FontSize', 12, 'FontWeight', 'bold');
    
    hold off;
    
    drawnow;

end