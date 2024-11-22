function eta_dd = eta_dyn(eta, eta_d, tau, params)
    % ETA_DYN computes the angular acceleration (eta_dd) of a drone
    % given its orientation (eta), angular velocity (eta_d), and torque input (tau).
    %
    % param eta: [roll; pitch; yaw] orientation angles (radians)
    % param eta_d: [roll_rate; pitch_rate; yaw_rate] angular velocities (rad/s)
    % param tau: [tau_roll; tau_pitch; tau_yaw] torque inputs in body frame (N·m)
    %
    % return: eta_dd - [roll_accel; pitch_accel; yaw_accel] angular accelerations (rad/s^2)

    % Compute the inertia matrix in the body frame
    J = inertia_matrix_J(eta, params); % J is 3x3
    disp("inv J is: ")
    disp(inv(J))
    % Compute the Coriolis matrix based on current orientation and angular velocity
    C = Coriolis(eta, eta_d, params); % C is 3x3
    
    disp("C is: ")
    disp(C)
    % Compute angular accelerations
    eta_dd = inv(J) * (tau - C * eta_d);
end


