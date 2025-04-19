function G_fires = fires_dens_function(dimgrid,pos_est_fire1,pos_fire2,sigma_fire1,sigma_fire2)

[x_m, y_m] = meshgrid(1:dimgrid(1), 1:dimgrid(2));
A1 = 1; % Amplitude of the first fire
A2 = 1; % Amplitude of the second fire

if sigma_fire1 <= 0
    sigma_fire1 = 1;
    A1 = 0; % Avoid division by zero
end
if sigma_fire2 <= 0
    sigma_fire2 = 1; 
    A2 = 0; % Avoid division by zero
end

G_fires = A1*exp(-(((x_m - pos_est_fire1(1)).^2) / (2 * sigma_fire1^2) + ((y_m - pos_est_fire1(2)).^2) / (2 * sigma_fire1^2))) + ...
          A2*exp(-(((x_m - pos_fire2(1)).^2) / (2 * sigma_fire2^2) + ((y_m - pos_fire2(2)).^2) / (2 * sigma_fire2^2)));