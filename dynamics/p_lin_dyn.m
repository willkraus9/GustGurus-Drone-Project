function pdd = p_lin_dyn(eta, pdot, params)
% Extract roll, pitch, and yaw from the vector eta
    roll = eta(1);
    pitch = eta(2);
    yaw = eta(3);
    
    % Extract velocity components
    xd = pdot(1);
    yd = pdot(2);
    zd = pdot(3);

    m = params.m;
    T = params.T;
    g = params.g;

    xdd = Ax * xd;
    ydd = Ay * yd;
    zdd = T/m - g + Az * zd;
end

