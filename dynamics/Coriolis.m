function C = Coriolis(eta, eta_d, params)
    % Coriolis computes the Coriolis matrix for a rigid body in 3D space.
    %
    % param eta: [roll; pitch; yaw] orientation angles (radians)
    % param eta_d: [roll_rate; pitch_rate; yaw_rate] angular velocity components (rad/s)
    %
    % return: C - 3x3 Coriolis matrix (N·m·s)

    % Extract roll, pitch, and yaw from the orientation vector eta
    roll = eta(1);   % Roll angle (phi)
    pitch = eta(2);  % Pitch angle (theta)
    yaw = eta(3);    % Yaw angle (psi)
    
    % Extract angular velocities (roll, pitch, yaw rates) from eta_d
    roll_d = eta_d(1);   % Roll rate (phi_dot)
    pitch_d = eta_d(2);  % Pitch rate (theta_dot)
    yaw_d = eta_d(3);    % Yaw rate (psi_dot)
    
    % Moments of inertia (must be predefined or passed globally)
    I11 = params.Ix; % Moment of inertia about x-axis
    I22 = params.Iy; % Moment of inertia about y-axis
    I33 = params.Iz; % Moment of inertia about z-axis

    % Trigonometric shorthand for efficiency
    s_phi = sin(roll);    % Sine of roll
    c_phi = cos(roll);    % Cosine of roll
    s_theta = sin(pitch); % Sine of pitch
    c_theta = cos(pitch); % Cosine of pitch
    
    % Coriolis matrix elements
    % Each element is derived from the equations of motion for a rigid body.
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
