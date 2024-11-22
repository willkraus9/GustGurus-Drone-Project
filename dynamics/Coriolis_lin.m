function C = Coriolis_lin(eta, eta_d, params)
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
    C = zeros(3, 3);
    C(1,1)=0;
    C(1,2) = (I33-I22)*psi_d - I11*psi_d;
    C(1,3) =0;
    C(2,1) =(I22-I33)*psi_d + I11*psi_d;
    C(2,2) =0;
    C(2,3) =0;
    C(3,1) = -I11*theta_d;
    C(3,2) = (I22-I33)*psi_d;

end

