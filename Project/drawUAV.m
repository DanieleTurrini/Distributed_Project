function drawUAV(X,Y,Z,Theta,dim,col)
    
    %Triangle-like unicycle
    s=dim;
    line([X+s*cos(Theta+(3*pi/5)),X+s*cos(Theta-(3*pi/5))],[Y+s*sin(Theta+(3*pi/5)), Y+s*sin(Theta-(3*pi/5))],[Z,Z],'Color',col);
    line([X+s*cos(Theta+(3*pi/5)), X+1.5*s*cos(Theta)],[Y+s*sin(Theta+(3*pi/5)),Y+1.5*s*sin(Theta)],[Z,Z], 'Color',col);
    line([X+1.5*s*cos(Theta), X+s*cos(Theta-(3*pi/5))],[Y+1.5*s*sin(Theta),Y+s*sin(Theta-(3*pi/5))],[Z,Z],'Color',col);
    %Caster/castor.
    line([X+s*cos(Theta), X+s*cos(Theta)],[Y+s*sin(Theta), Y+s*sin(Theta)],[Z,Z],'Color',col);
    %Centre of the axis.
    line([X, X],[Y, Y],[Z,Z],'Color',col);
    
    % Draw a vertical line from the ground (0) to the UAV (Z)
    line([X, X], [Y, Y], [0, Z], 'Color', col, 'LineStyle', '--', 'LineWidth', 0.5);
     
end