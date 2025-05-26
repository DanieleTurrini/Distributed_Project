clear all
close
clc

load test.mat;


if UAV_FAIL

    numUAV = numUAV + 1;
      
end

figure_size = [300, 80, 900, 700]; % Set figure size

%% Plot 2D voronoi simulation

figure(50)
set(gcf, 'Position', figure_size);
bx = subplot(1,1,1);
axis(bx,[0 dimgrid(1) 0 dimgrid(2)]);
xlabel(bx,'X Coordinate');
ylabel(bx,'Y Coordinate');
view(bx,2);

for t = 1:2:count

    figure(50)
    cla(bx);
    title(bx,['2D Voronoi Simulation - Iteration:', num2str(t),'/', num2str(tot_iter)]);
    cla;
    hold(bx,'on');

    if sigmaFir1StoreReal(1,t) > 10 % If fire is extinguished, do not plot the X

        plot(bx,posFir1StoreReal(1,1,t), posFir1StoreReal(1,2,t), 'rx', 'MarkerSize', sigmaFir1StoreReal(1,t));

    end

    if sigmaFir2StoreReal(1,t) > 10 % If fire is extinguished, do not plot the X

        plot(bx,posFir2StoreReal(1,1,t), posFir2StoreReal(1,2,t), 'rx', 'MarkerSize', sigmaFir2StoreReal(1,t));

    end

    plot(bx,x_water,y_water,'o','Color', 'b', 'MarkerSize', 90);

    for i = 1:numUAV

        % Draw the UAV pose
        drawUAV2D(trajectories(i, 1,t), trajectories(i, 2,t), trajectories(i, 4,t), dim_UAV, 'k');
        drawUAV2D(trajectories_est(i, 1,t), trajectories_est(i, 2,t), trajectories_est(i, 4,t), dim_UAV,'g');

        if SAFETY_VORONOI

            phi = linspace(0, 2*pi, 100);
            h = plot(bx, trajectories_est(i, 1,t) + deltaSafety*cos(phi), trajectories_est(i, 2,t) + deltaSafety*sin(phi), '--', 'Color', [0.5, 0.7, 0.2], 'LineWidth', 0.25);
            hold on;
            % Imposta la trasparenza (alpha) della linea
            if isprop(h, 'Color')
                h.Color(4) = 0.3; % Valore alpha tra 0 (trasparente) e 1 (opaco)
            end

            traj = squeeze(virtual_trajectories(:,:,i,t));
            plot(bx, traj(:,1), traj(:,2), 'o');

            % for j = 1:numUAV
            %     plot(bx, virtual_trajectories(j+numUAV-1, 1, t), virtual_trajectories(j+numUAV-1, 2, t), 'ro', 'MarkerSize', 4);
            % end
        end
        
        plot(bx,centroids_est_stor(i,1,t), centroids_est_stor(i,2,t), 'x', 'Color',[0.4660, 0.6740, 0.1880]);

        if Fir1Store(i,3,t) > 0

            plot(bx,Fir1Store(i,1,t), Fir1Store(i,2,t), 'x','MarkerSize', Fir1Store(i,3,t),'Color',[0.4940, 0.1840, 0.5560]);

        end

        if Fir2Store(i,3,t) > 0

            plot(bx,Fir2Store(i,1,t), Fir2Store(i,2,t), 'x','MarkerSize', Fir2Store(i,3,t),'Color',[0.4940, 0.1840, 0.5560]);

        end
        
    end

    vx_es = vx_Data{t};
    vy_es = vy_Data{t};
    plot(bx,vx_es, vy_es,'Color',[0.4660, 0.6740, 0.1880]);

    % Plot Voronoi edges virtual
    if SAFETY_VORONOI
        
        h_vor = plot(bx, vx_virt_Data{t}, vy_virt_Data{t}, 'Color', [1, 0, 0 0.2]); % Rosso flebile, trasparente
        %set(h_vor, 'Color', [1, 0, 0, 0.2]); % RGBA, alpha=0.2
    end    
    hold(bx,'off');
    drawnow; 
    
end   


%% Plot 3D Simulation

figure(60);
set(gcf, 'Position', figure_size);
ax = subplot(1,1,1);
axis(ax,[0 dimgrid(1) 0 dimgrid(2) 0 dimgrid(3)]);
hold(ax,'on');

% Loading texture
texture = imread('Mountain.jpg');  % Must be RGB
texture_resized = imresize(texture, [size(Xf,1), size(Xf,2)]);
texture_resized = double(texture_resized) / 255;  % Normalize between 0 and 1

% Increase brightness
brightness_factor = 1.5;
texture_resized = min(texture_resized * brightness_factor, 1);

% Create a uniform green background
green_background = repmat(reshape([0.4660 0.6740 0.1880], 1, 1, 3), size(texture_resized, 1), size(texture_resized, 2));

% Transparency factor (0 = only green background, 1 = only texture)
alpha = 0.2;  

% Blend texture and background
blended_texture = (alpha * texture_resized + (1 - alpha) * green_background);

% static environment 
surf(ax, Xf, Yf, Zf, 'CData', flip(blended_texture, 1), ...
    'FaceColor', 'texturemap', ...
    'EdgeColor', 'none', ...
    'FaceLighting', 'none');

lighting none;

contour3(ax, Xf, Yf, Zf, 20, 'k');

% Static water circle 
theta = linspace(0,2*pi,100);
r = sigma_water/2 + 20 ;
x_circle = r*cos(theta)+x_water;
y_circle = r*sin(theta)+y_water;
z_circle = 5 * ones(size(theta));
hWater = plot3(ax,x_circle, y_circle, z_circle, 'b', 'LineWidth', 2);

view(ax,3);
grid(ax,'on');
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');

% simple airplane body‑frame
fuselage = [  1;  0;  0 ];        % nose
wingL    = [ 0;  0.5;  0 ];      % left wingtip
wingR    = [ 0; -0.5;  0 ];      % right wingtip
tailT    = [ -0.5; 0;   0.2 ];    % tail top
tailB    = [ -0.5; 0;  -0.2 ];    % tail bottom

% vertices matrix
V = [ fuselage, wingL, wingR, tailT, tailB ]';  

% faces list (triangles)
F = [ ...
    1 2 3;   % main wing
    1 4 5;   % tailplane top/bottom
    2 4 3;   % left side fuselage
    3 5 2;   % right side fuselage
];

V = dim_UAV * V;

hHeightLine = gobjects(numUAV,1);

% number of UAVs
hPlaneTF = gobjects(numUAV,1);
for i = 1:numUAV
   
    hPlaneTF(i) = hgtransform;

    patch(ax, 'Vertices', V, 'Faces', F, ...
           'FaceColor', rand(1,3)*0.5 + 0.5, ...
           'EdgeColor', 'k', ...
           'Parent', hPlaneTF(i) );

    hHeightLine(i) = plot3(ax,[NaN NaN], [NaN NaN], [NaN NaN], ...  
                            ':', ...      
                            'Color', [0 0 0], ...
                            'LineWidth', 0.2 );

end

% placeholders for the real‐fire and estimated‐fire markers
hFireReal1 = plot3(ax,NaN, NaN, NaN, 'xr', 'LineWidth', 2);
hFireEst1  = plot3(ax,NaN, NaN, NaN, 'x',  'MarkerSize', 8);    
hFireReal2 = plot3(ax,NaN, NaN, NaN, 'xr', 'LineWidth', 2);
hFireEst2  = plot3(ax,NaN, NaN, NaN, 'x',  'MarkerSize', 8);


for t = 1:count
    title(['3D Simulation - Iteration:', num2str(t),'/', num2str(tot_iter)]);

    % Update each UAV’s position
    for i = 1:numUAV

        x = trajectories(i,1,t);
        y = trajectories(i,2,t);
        z = trajectories(i,3,t);
        theta = trajectories(i,4,t); 

        % surface height under the UAV    
        z0  = environment_surface(x, y, 1);

        T = makehgtform('translate',[x,y,z], ...
                        'zrotate', theta);

        set(hPlaneTF(i), 'Matrix', T);

        set(hHeightLine(i), ...
            'XData', [x,    x], ...
            'YData', [y,    y], ...
            'ZData', [z0,   z] );
    end

    % Update the “real” fire1 marker 
    if sigmaFir1StoreReal(1,t) > 0
        set(hFireReal1, ...
            'XData', posFir1StoreReal(1,1,t), ...
            'YData', posFir1StoreReal(1,2,t), ...
            'ZData', environment_surface(...
                    posFir1StoreReal(1,1,t), ...
                    posFir1StoreReal(1,2,t), 1), ...
            'MarkerSize', sigmaFir1StoreReal(1,t));
    else
        set(hFireReal1, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    end

    % Update the “estimated” fire1 marker
    if Fir1Store(i,3,t) > 0
        set(hFireEst1, ...
            'XData', Fir1Store(i,1,t), ...
            'YData', Fir1Store(i,2,t), ...
            'ZData', environment_surface(...
                    Fir1Store(i,1,t), ...
                    Fir1Store(i,2,t), 1), ...
                    'MarkerSize', Fir1Store(i,3,t),...
                    'MarkerEdgeColor', [1.0, 0.5, 0.0]);
    else 
        set(hFireEst1, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    end
    % Update the “real” fire2 marker
    if sigmaFir2StoreReal(1,t) > 0
        set(hFireReal2, ...
            'XData', posFir2StoreReal(1,1,t), ...
            'YData', posFir2StoreReal(1,2,t), ...
            'ZData', environment_surface(...
                    posFir2StoreReal(1,1,t), ...
                    posFir2StoreReal(1,2,t), 1), ...
            'MarkerSize', sigmaFir2StoreReal(1,t));
    
    else 
        set(hFireReal2, 'XData', NaN, 'YData', NaN, 'ZData', NaN);    
    end
    % Update the “estimated” fire2 marker 
    if Fir2Store(i,3,t) > 0
        set(hFireEst2, ...
            'XData', Fir2Store(i,1,t), ...
            'YData', Fir2Store(i,2,t), ...
            'ZData', environment_surface(...
                    Fir2Store(i,1,t), ...
                    Fir2Store(i,2,t), 1), ...
            'MarkerSize', Fir2Store(i,3,t),...
            'MarkerEdgeColor', [1.0, 0.5, 0.0]); % Orange color
    else
        set(hFireEst2, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    end

    hold(ax,'off');
    drawnow; 

    pause(0.0025); % Pause to control the speed of the animation
    
end


