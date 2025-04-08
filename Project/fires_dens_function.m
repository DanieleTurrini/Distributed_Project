function G_fires = fires_dens_function(dimgrid,pos_est_fire1,pos_fire2,sigma_fire1,sigma_fire2)

[x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));

G_fires = exp(-(((x_m - pos_est_fire1(1)).^2) / (2 * sigma_fire1^2) + ((y_m - pos_est_fire1(2)).^2) / (2 * sigma_fire1^2))) + ...
          exp(-(((x_m - pos_fire2(1)).^2) / (2 * sigma_fire2^2) + ((y_m - pos_fire2(2)).^2) / (2 * sigma_fire2^2)));