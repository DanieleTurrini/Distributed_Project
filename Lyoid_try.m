clear all;
close all;
clc;

% Define the number of points and grid dimensions
numPoints = 4;      % Set the number of points you want to generate
dimgrid = [100 100];   % Define the height of the grid

points = zeros(numPoints,2);

% Generate random positions for each point
x = rand(numPoints, 1) * dimgrid(1);   % Random x coordinates
y = rand(numPoints, 1) * dimgrid(2);  % Random y coordinates

points = [x,y];

% Plot the points on a figure
figure(1);
scatter(points(:,1),points(:,2));
axis([0 dimgrid(1) 0 dimgrid(2)]); 
xlabel('X Coordinate');
ylabel('Y Coordinate');
title(sprintf('Randomly Placed %d Points', numPoints));

% Compute and plot Voronoi tessellation
[vx, vy] = voronoi(points(:,1), points(:,2));

figure(2);
hold on;
plot(vx, vy, 'r-', 'LineWidth', 1.5);
axis([0 dimgrid(1) 0 dimgrid(2)]); 
scatter(points(:,1),points(:,2));
title('Voronoi Tassellation');

%% Compute Voronoi tessellation using voronoin
[V, C] = voronoin(points);

%% For simplicity, assume that every cell is fully bounded inside the grid.
% (For unbounded cells, you need to clip the polygon to the grid first.)
density = 1;  % mass per unit area
cellMasses = zeros(numPoints, 1);

for i = 1:numPoints
    cellIdx = C{i};
    
    % Check for unbounded cell (index 1 is at infinity)
    if any(cellIdx==1)
        fprintf('Cell %d is unbounded; it must be clipped before integration.\n', i);
        continue;
    end
    
    % Extract the vertices of the cell (assumed in order)
    xv = V(cellIdx, 1);
    yv = V(cellIdx, 2);
    
    % Compute area using the polyarea function (integration over the polygon)
    cellArea = polyarea(xv, yv);
    
    % Mass is density multiplied by area
    cellMasses(i) = density * cellArea;
    
    % Optionally, plot the cell for visualization
    hold on;
    plot([xv; xv(1)], [yv; yv(1)], 'r-', 'LineWidth', 1.5);
end

%% Display computed masses
fprintf('Mass of each Voronoi cell (for bounded cells):\n');
for i = 1:numPoints
    if cellMasses(i) > 0
        fprintf('Cell %d: %.2f\n', i, cellMasses(i));
    end
end