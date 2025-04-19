function [X, Y, Z] = plot_enviroment_surface(bool)
    % Define grid range
    x_range = linspace(0, 500, 200); % 200 points for smoothness
    y_range = linspace(0, 500, 200);
    
    % Create meshgrid
    [X, Y] = meshgrid(x_range, y_range);
    
    % Compute height for each (X, Y)
    Z = arrayfun(@(x, y) enviroment_surface(x, y, 1), X, Y);
    if bool
        % Plot the surface
        figure;
        surf(X, Y, Z);
        shading interp;
        colormap jet;
        colorbar;
        xlabel('X');
        ylabel('Y');
        zlabel('Height');
        title('Smooth Flight Surface');
        axis([0 500 0 500 0 500]);
    end
end