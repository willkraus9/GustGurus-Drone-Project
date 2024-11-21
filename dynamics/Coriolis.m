function C = Coriolis(eta, eta_d)
    % Extract roll, pitch, and yaw from the vector eta
    roll = eta(1);
    pitch = eta(2);
    yaw = eta(3);
    
    % Extract roll, pitch, and yaw rates from the vector eta_d
    roll_d = eta_d(1);
    pitch_d = eta_d(2);
    yaw_d = eta_d(3);
    
    % Moments of inertia
    I11 = Ix;
    I22 = Iy;
    I33 = Iz;
    
    % Trigonometric shorthand
    s_phi = sin(roll);
    c_phi = cos(roll);
    s_theta = sin(pitch);
    c_theta = cos(pitch);
    
    % Coriolis matrix elements
    C11 = 0;
    C12 = -((I33 - I22) * (yaw_d * c_phi * s_theta + pitch_d * (c_theta^2 - c_phi^2)) - I11 * roll_d * c_phi);
    C13 = ((I22 - I33) * pitch_d * s_phi + yaw_d * (c_theta^2 - s_phi^2)) + I11 * roll_d * s_phi;
    C21 = ((I33 - I22) * yaw_d * s_phi + roll_d * (c_theta^2 - s_phi^2)) + I11 * roll_d * c_phi;
    C22 = ((I33 - I22) * yaw_d * c_phi * s_theta);
    C23 = (-I11 * roll_d + I22 * pitch_d * s_phi * c_theta + I33 * yaw_d) * c_phi * s_theta;
    C31 = ((I22 - I33) * yaw_d * (c_phi * s_theta * c_theta - c_phi * roll_d) - I11 * roll_d);
    C32 = ((I33 - I22) * (yaw_d * c_phi * s_theta * c_theta + roll_d * (c_phi * (s_theta^2 - c_theta^2))) + (I11 - I33) * roll_d * c_phi);
    C33 = ((I22 - I33) * yaw_d * c_phi * s_theta * c_theta + (I11 - I22) * roll_d * s_theta^2 - I33 * roll_d * c_phi * s_theta);
    
    % Construct the Coriolis matrix
    C = [C11, C12, C13;
         C21, C22, C23;
         C31, C32, C33];
end
