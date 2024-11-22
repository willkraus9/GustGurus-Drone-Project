function pdd = p_dyn(eta, pdot, T)
    % Extract roll, pitch, and yaw from the vector eta
    roll = eta(1);
    pitch = eta(2);
    yaw = eta(3);
    
    % Extract velocity components
    xd = pdot(1);
    yd = pdot(2);
    zd = pdot(3);
    
    
    % Trigonometric shorthand
    c_phi = cos(roll);
    s_phi = sin(roll);
    c_theta = cos(pitch);
    s_theta = sin(pitch);
    c_psi = cos(yaw);
    s_psi = sin(yaw);
    
    xdd = T/m * (cos(pitch) * sin(yaw) + sin(roll) * sin(pitch) * cos(yaw)) + Ax * xd;
    ydd = T/m * (sin(pitch) * sin(roll) - cos(roll) * sin(pitch) * cos(yaw)) + Ay * yd;
    zdd = T/m * (-cos(pitch) * cos(yaw)) - g + Az * zd;
    
    % Combine into a vector
    pdd = [x_dd; y_dd; z_dd];
end
