function J = inertia_matrix_J(eta, I_B)
    % param eta: roll, pitch, yaw vector
    % param I_B: Inertia tensor in Body frame
    % return J: Inertia tensor in world frame
    W = bigW(eta);

    J = W.T * I_B * W;
end

