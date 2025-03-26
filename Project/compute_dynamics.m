function next_state = compute_dynamics(A,B,current_state,control,dt)
    
    % Dynamics of our system
    next_state_row = A * current_state' + B(current_state(4),dt) * control';
    next_state = next_state_row';