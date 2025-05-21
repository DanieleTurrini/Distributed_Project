% ENVIRONMENT_SURFACE Computes the surface height at given (x, y) coordinates for a specified scenario.
%   height = ENVIRONMENT_SURFACE(x, y, scenario) returns the height of the environment surface
%   at the coordinates (x, y) based on the selected scenario. For scenario 1, the surface is
%   modeled as a sum of Gaussian functions. For other scenarios, the height is set to zero.

function height = environment_surface(x, y, scenario)
    % Define Gaussian function parameters
    if scenario == 1
        height = 200 * exp(-(((x - 300).^2) / (2 * 100^2) + ((y - 450).^2) / (2 * 100^2))) + ...
                 80 * exp(-(((x - 250).^2) / (2 * 50^2) + ((y - 100).^2) / (2 * 150^2)));
    else
        height = 0; % Default case
    end
end