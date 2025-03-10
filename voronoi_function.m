function [areas,centroids,vel] = voronoi_function(Map, c_points, kp)

[X, Y] = meshgrid(1:Map(1), 1:Map(2));
voronoi_grid = [X(:), Y(:)];

c_x = c_points(:,1);
c_y = c_points(:,2);

% Compute distances between starting points and each point on the grid
distances = pdist2(voronoi_grid, c_points);

% Find closer starting point index for each point on the grid
[~, minimum_indices] = min(distances, [], 2);

% Assign index to each point on the grid
indices_cell = reshape(minimum_indices, Map);

areas = zeros(length(c_x),1);
centroids = zeros(length(c_x),2);
vel = zeros(length(c_x),2);

for i = 1:length(c_x)
    % Estrai i punti della regione assegnata al drone i
    region_points = voronoi_grid(minimum_indices == i, :);

    areas(i) = size(region_points, 1);

    centroids(i,:) = round(mean(region_points));

    vel(i, :) = kp * (centroids(i, :) - c_points(i, :));

end

% Plot della Voronoi
figure(3)
imagesc(indices_cell);
hold on;
scatter(centroids(:,1),centroids(:,2),60)
% scatter(c_points(:,1),c_points(:,2),60,'x')

end