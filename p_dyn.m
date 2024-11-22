function pdd = p_dyn(eta, pdot, T)
    % Extract roll, pitch, and yaw from the vector eta
    phi = eta(1);
    theta = eta(2);
    psi = eta(3);
    
    % Extract velocity components
    xd = pdot(1);
    yd = pdot(2);
    zd = pdot(3);
    
    
    % Trigonometric shorthand
    c_phi = cos(phi);
    s_phi = sin(phi);
    c_theta = cos(theta);
    s_theta = sin(theta);
    c_psi = cos(psi);
    s_psi = sin(psi);
    
     % Gravity vector
    g_vec = [0; 0; -g];
    
    % Thrust vector in inertial frame
    thrust_vec = (T / m) * [
        c_psi * s_theta * c_phi + s_psi * s_phi;
        s_psi * s_theta * c_phi - c_psi * s_phi;
        c_theta * c_phi
    ];
    
    % Aerodynamic drag vector
    drag_vec = (1 / m) * [
        Ax * xd;
        Ay * yd;
        Az * zd
    ];
    
    % Compute acceleration vector
    pdd = g_vec + thrust_vec - drag_vec;

end
