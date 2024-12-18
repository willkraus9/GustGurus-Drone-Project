function W = bigW(eta)
    % W(eta) is the mapping from eta_d to w_B
    % eta_d is velocity of roll pitch yaw
    % w_b (omega_B) is the angular velocity in the body coordinate system
    % Extract roll and pitch from the vector eta
    roll = eta(1);
    pitch = eta(2);
    
    % Construct the matrix W
    W = [1 0 -sin(roll);
         0 cos(pitch) cos(roll);
         0 -sin(pitch) cos(pitch)];
end


