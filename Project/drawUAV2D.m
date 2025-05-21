% drawUAV2D Draws a 2D representation of a UAV as a triangle-like unicycle.
%   drawUAV2D(X, Y, Theta, dim, col) plots the UAV at position (X, Y) with
%   orientation Theta, size dim, and color col.

function drawUAV2D(X,Y,Theta,dim,col)
    
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