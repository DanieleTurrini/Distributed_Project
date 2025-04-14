function Z = flatToppedGaussian(X, Y, center, width, flatWidth)
% flatToppedGaussian3D generates a smooth flat-topped Gaussian-like function in 3D.
%
%   Z = flatToppedGaussian3D(X, Y, center, width, flatWidth)
%
%   Inputs:
%     X, Y      - Meshgrid matrices representing the x and y coordinates.
%     center    - A 1x2 vector [centerX, centerY] specifying the center of the peak.
%     width     - Controls the smoothness of the transition edges (larger means smoother).
%     flatWidth - The diameter of the flat top (plateau) region.
%
%   Output:
%     Z         - A matrix of the same size as X and Y, containing the computed height.

    % Compute the radial distance from the center for each (x,y) point.
    dx = X - center(1);
    dy = Y - center(2);
    r = sqrt(dx.^2 + dy.^2);
    
    % Define the transition zone.
    % Points with r <= flatWidth/2 will be in the flat top,
    % while the smooth fall-off occurs for r > flatWidth/2.
    transition = (r - flatWidth/2) / (width/2);
    
    % Use a hyperbolic tangent to create a smooth edge.
    % The function outputs values near 1 for low r and falls off smoothly.
    Z =  (1 - tanh(transition));
    Z(r <= flatWidth/2) = 1;

end
