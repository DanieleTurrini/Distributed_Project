% Define a grid.
x = linspace(-5, 5, 200);
y = linspace(-5, 5, 200);
[X, Y] = meshgrid(x, y);

% Set parameters: center at (0,0), transition smoothness factor and plateau width.
center = [0, 0];
width = 1;      % Adjust this to control edge smoothness.
flatWidth = 5;  % Adjust this to change the diameter of the flat top.

% Evaluate the flat-topped Gaussian function.
Z = - flatToppedGaussian(X, Y, center, width, flatWidth);

% Visualize the result.
figure;
surf(X, Y, Z, 'EdgeColor', 'none');  % Create a smooth surface plot.
colormap jet;
colorbar;
title('Smooth Flat-Topped Gaussian in 3D');
xlabel('X');
ylabel('Y');
zlabel('Z');
view(45, 30);  % Set a good view angle.
