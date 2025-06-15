% UAV_CONTROL Computes the velocity command for a UAV to move towards a target centroid.
%   vel = UAV_control(weigth_centroids, state, Kp, Ka) calculates the linear and angular
%   velocities required for the UAV to reach the specified centroid position using proportional
%   control gains for position (Kp) and angle (Ka).

function vel = UAV_control(weigth_centroids, state, Kp, Ka)
     
    distances = norm(weigth_centroids - state(1:2));

    % Angle from centroid
    angle_to_goal = atan2(weigth_centroids(2) - state(2), ...
                          weigth_centroids(1) - state(1));
    
    % Error angle
    angle_error = angle_to_goal - state(4); 
    angle_error = wrapToPi(angle_error);
    
    % Linear velocity 
    v_lin = Kp * distances;

    % Angular velocity 
    w_ang = Ka * (angle_error);

    % Velocity vector
    vel = [v_lin, 0, w_ang];
    