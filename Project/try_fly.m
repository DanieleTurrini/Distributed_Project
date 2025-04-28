% x_range = linspace(0, 500, 200); % 200 points for smoothness
% y_range = linspace(0, 500, 200);
% 
% % Create meshgrid
% [X, Y] = meshgrid(x_range, y_range);
% 
% % Compute height for each (X, Y)
% Z = arrayfun(@(x, y) flight_surface(x, y, 30, 1), X, Y);
% 
% disp(flight_surface(50, 50, 30, 1));
% 
% % Plot the surface
% figure;
% surf(X, Y, Z);
% shading interp;
% colormap jet;
% colorbar;
% xlabel('X');
% ylabel('Y');
% zlabel('Height');
% title('Smooth Flight Surface');
% axis([0 500 0 500 0 500]);

[G_fire, G_water] = objective_density_functions([500 500 500], pos_fire1_mov, pos_fire2_mov, pos_water, sigma_fire1, sigma_fire2, sigma_water, 0, PLOT_DENSITY_FUNCTIONS);