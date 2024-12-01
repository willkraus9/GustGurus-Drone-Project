function Tb = Baseline(eta)
%BASELINE Summary of this function goes here
%   Detailed explanation goes here
phi = eta(1);
theta = eta(2);
psi = eta(3);
Tb=9.81*params.m*cos(phi)*cos(theta);
end

