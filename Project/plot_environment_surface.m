% PLOT_ENVIRONMENT_SURFACE Plots a 3D surface representing the environment map.
%   [X, Y, Z] = PLOT_ENVIRONMENT_SURFACE(BOOL) generates a meshgrid over a
%   specified range and computes the surface heights using the environment_surface
%   function. If BOOL is true, it visualizes the surface with additional features
%   such as fire locations, a water region, and contour lines for enhanced
%   visualization. Returns the meshgrid coordinates X, Y, and the computed heights Z.

function [X, Y, Z] = plot_environment_surface(dimgrid, PLOT_ENVIRONMENT)
    % Define grid range
    x_range = linspace(0, dimgrid(1), 300); 
    y_range = linspace(0, dimgrid(2), 300);
    
    % Create meshgrid
    [X, Y] = meshgrid(x_range, y_range);
    
    % Compute height for each (X, Y)
    Z = arrayfun(@(x, y) environment_surface(x, y, 1), X, Y);
    
    if PLOT_ENVIRONMENT
    
        % Plot the green surface
        figure;
        surf(X, Y, Z, ...
            'FaceColor', [0.4660 0.6740 0.1880], ...
            'EdgeColor', 'none', ...
            'FaceAlpha', 0.9);

        theta = linspace(0, 2 * pi, 100); % Precomputed angular parameter
        r =  45;
        x_circle = r * cos(theta) + 50;
        y_circle = r * sin(theta) + 50;
        z_circle = ones(size(theta)) * 5;

        hold on;

        % Plot the fires
        plot3(300, 400, environment_surface(300, 400, 1), 'x', 'Color', 'r', 'MarkerSize', 50, 'LineWidth', 3);
        plot3(450, 50, environment_surface(450, 50, 1), 'x', 'Color', 'r', 'MarkerSize', 20, 'LineWidth', 3);

        % Plot the water
        plot3(x_circle, y_circle, z_circle, 'b', 'LineWidth', 2);
        
        % Add contour lines at Z-levels, projected onto the xy-plane
        contour3(X, Y, Z, 20, ...    % 20 contour levels
            'LineWidth', 1, ...
            'LineColor', 'k');       % black

        
        
        shading interp;
        colormap([0.4660 0.6740 0.1880]);  
        colorbar;
        xlabel('X');
        ylabel('Y');
        zlabel('Height');
        title('Environment Map');
        axis([0 dimgrid(1) 0 dimgrid(2) 0 dimgrid(3)]);
        view(3);
        hold off;

    end
    
end
