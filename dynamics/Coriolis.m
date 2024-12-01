function C = Coriolis(eta, eta_d, params)
    % Extract roll, pitch, and yaw from the vector eta
    phi = eta(1);   % Roll angle
    theta = eta(2); % Pitch angle
    psi = eta(3);   % Yaw angle
    
    % Extract roll, pitch, and yaw rates from the vector eta_d
    phi_d = eta_d(1);   % Roll rate
    theta_d = eta_d(2); % Pitch rate
    psi_d = eta_d(3);   % Yaw rate
    
    % Moments of inertia
    I11 = params.Ix;
    I22 = params.Iy;
    I33 = params.Iz;
    
    % Trigonometric shorthand
    s_phi = sin(phi);
    c_phi = cos(phi);
    s_theta = sin(theta);
    c_theta = cos(theta);
    
    % Initialize Coriolis matrix
    C = zeros(3, 3);

    % Populate Coriolis matrix using the formulas from the image
    C(1, 1) = 0;
    C(1, 2) = (I22 - I33) * (theta_d * c_phi * s_phi + psi_d * c_theta * (s_phi^2 - c_phi^2)) - I11 * psi_d * c_theta;
    C(1, 3) = (I33 - I22) * psi_d * c_phi * s_phi * c_theta^2;
    C(2, 1) = (I33 - I22) * (theta_d * c_phi * s_phi + psi_d * c_theta * (s_phi^2 - c_phi^2)) + I11 * psi_d * c_theta;
    C(2, 2) = (I33 - I22) * phi_d * c_phi * s_phi;
    C(2, 3) = (-I11 * psi_d + I22 * psi_d * s_phi^2 + I33 * psi_d * c_phi^2) * s_theta * c_theta;
    C(3, 1) = (I22 - I33) * psi_d * c_theta^2 * s_phi * c_phi - I11 * theta_d * c_theta;
    C(3, 2) = (I33 - I22) * (theta_d * c_phi * s_phi * s_theta + phi_d * c_theta * (s_phi^2 - c_phi^2)) + (I11 - I22 * s_phi^2 - I33 * c_phi^2) * psi_d * s_theta * c_theta;
    C(3, 3) = (I22 - I33) * phi_d * c_phi * s_phi * c_theta^2 + (I11 - I22 * s_phi^2 - I33 * c_phi^2) * theta_d * c_theta * s_theta;

end
