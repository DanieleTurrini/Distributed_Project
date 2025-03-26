A = [1, 0, 0;
     0, 1, 0;
     0, 0, 1];

B = @(theta) [ dt * cos(theta), 0;
               dt * sin(theta), 0;
               0,               dt];

dt = 1;
stato = [2,3,pi/2];
controllo = [5,0];

stato = stato * A'+ controllo * B(stato(3))'