function height = flight_surface(x, y, h, scenario)

    % Define Gaussian function parameters
    if scenario == 1
        
        x_w = 50;
        y_w = 50;
        flatWidth = 50;
        width = 80;

        dx = x - x_w;
        dy = y - y_w;
        r = sqrt(dx.^2 + dy.^2);

        transition = (r - flatWidth/2) / (width/2);
        
        Z = (1 - tanh(transition));
        Z(r <= flatWidth/2) = 1; 

        height = 200 * exp(-(((x - 300).^2) / (2 * 100^2) + ((y - 450).^2) / (2 * 100^2))) + ...
                 80 * exp(-(((x - 250).^2) / (2 * 50^2) + ((y - 100).^2) / (2 * 150^2))) + h - h * Z;
    else
        height = 0; % Default case
    end