% drawUAV2D Draws a 2D representation of a UAV as a triangle-like unicycle.
%   drawUAV2D(X, Y, Theta, dim, col) plots the UAV at position (X, Y) with
%   orientation Theta, size dim, and color col.

function drawUAV2D(X,Y,Theta,dim,col, deltaSafety)

    if nargin >= 6 && ~isempty(deltaSafety)
        phi = linspace(0, 2*pi, 100);
        h = plot(X + deltaSafety*cos(phi), Y + deltaSafety*sin(phi), '--', 'Color', [0.5, 0.7, 0.2], 'LineWidth', 0.25);
        hold on;
        % Imposta la trasparenza (alpha) della linea
        if isprop(h, 'Color')
            h.Color(4) = 0.3; % Valore alpha tra 0 (trasparente) e 1 (opaco)
        end
    end

    %Triangle-like unicycle
    s=dim;
    line([X+s*cos(Theta+(3*pi/5)),X+s*cos(Theta-(3*pi/5))],[Y+s*sin(Theta+(3*pi/5)), Y+s*sin(Theta-(3*pi/5))],'Color',col);
    line([X+s*cos(Theta+(3*pi/5)), X+1.5*s*cos(Theta)],[Y+s*sin(Theta+(3*pi/5)),Y+1.5*s*sin(Theta)], 'Color',col);
    line([X+1.5*s*cos(Theta), X+s*cos(Theta-(3*pi/5))],[Y+1.5*s*sin(Theta),Y+s*sin(Theta-(3*pi/5))],'Color',col);
    %Caster/castor.
    line([X+s*cos(Theta), X+s*cos(Theta)],[Y+s*sin(Theta), Y+s*sin(Theta)],'Color',col);
    %Centre of the axis.
    line([X, X],[Y, Y],'Color',col);
    
end