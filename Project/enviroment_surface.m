function height = enviroment_surface(x, y, scenario)
    % Define Gaussian function parameters
    if scenario == 1
        height = 200 * exp(-(((x - 300).^2) / (2 * 100^2) + ((y - 450).^2) / (2 * 100^2))) + ...
                 80 * exp(-(((x - 250).^2) / (2 * 50^2) + ((y - 100).^2) / (2 * 150^2)));
    else
        height = 0; % Default case
    end
end