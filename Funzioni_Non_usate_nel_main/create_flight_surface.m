function G_flight = create_flight_surface(dimgrid, scenario)

    % Creazione della griglia di punti
    [x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));

    if scenario == 1
        G_flight = 200 * exp(-(((x_m - 300).^2) / (2 * 100^2) + ((y_m - 450).^2) / (2 * 100^2))) + ...
        80 * exp(-(((x_m - 250).^2) / (2 * 80^2) + ((y_m - 100).^2) / (2 * 150^2)));
    end

    figure(2)
    surf(x_m, y_m, G_flight);
    shading interp;
    colormap jet;
    colorbar;
    axis([0 dimgrid(1) 0 dimgrid(2) 0 dimgrid(3)]);
    xlabel('X');
    ylabel('Y');
    zlabel('Height');
    title('Fligth Surface');
