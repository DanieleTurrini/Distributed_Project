function drawUnicycle(X,Y,Theta)
    
    %Triangle-like unicycle
    s=3;
    line([X+s*cos(Theta+(3*pi/5)),X+s*cos(Theta-(3*pi/5))],[Y+s*sin(Theta+(3*pi/5)), Y+s*sin(Theta-(3*pi/5))],'Color','k');
    line([X+s*cos(Theta+(3*pi/5)), X+1.5*s*cos(Theta)],[Y+s*sin(Theta+(3*pi/5)),Y+1.5*s*sin(Theta)], 'Color','k');
    line([X+1.5*s*cos(Theta), X+s*cos(Theta-(3*pi/5))],[Y+1.5*s*sin(Theta),Y+s*sin(Theta-(3*pi/5))],'Color','k');
    %Caster/castor.
    line([X+s*cos(Theta), X+s*cos(Theta)],[Y+s*sin(Theta), Y+s*sin(Theta)],'Color','k');
    %Centre of the axis.
    line([X, X],[Y, Y],'Color','k');
     
end