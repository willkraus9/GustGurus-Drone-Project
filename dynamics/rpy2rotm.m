function R = rpy2rotm(eta)
    % param eta: roll pitch yaw vector
    % return R: Roation MAtrix
    
    % Extract roll, pitch, and yaw from the vector eta
    roll = eta(1);
    pitch = eta(2);
    yaw = eta(3);
    
    % Rotation matrix for roll
    R_roll = [1 0 0;
              0 cos(roll) -sin(roll);
              0 sin(roll) cos(roll)];
    
    % Rotation matrix for pitch
    R_pitch = [cos(pitch) 0 sin(pitch);
               0 1 0;
               -sin(pitch) 0 cos(pitch)];
    
    % Rotation matrix for yaw
    R_yaw = [cos(yaw) -sin(yaw) 0;
             sin(yaw) cos(yaw) 0;
             0 0 1];
    
    % Combined rotation matrix
    R = R_yaw * R_pitch * R_roll;

end

