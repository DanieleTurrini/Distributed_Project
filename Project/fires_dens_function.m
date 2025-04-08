function G_fires = fires_dens_function(pos,pos_fire1,pos_fire2,sigma_fire1,sigma_fire2,t)

posFireMov1 = pos_fire1(t);

G_fires = exp(-(((pos(1) - posFireMov1(1)).^2) / (2 * sigma_fire1^2) + ((pos(2) - posFireMov1(2)).^2) / (2 * sigma_fire1^2))) + ...
          exp(-(((pos(1) - pos_fire2(1)).^2) / (2 * sigma_fire2^2) + ((pos(2) - pos_fire2(2)).^2) / (2 * sigma_fire2^2)));