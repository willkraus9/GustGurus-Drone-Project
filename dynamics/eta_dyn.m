function etadd = eta_dyn(eta, eta_d, tau)
%ETA_DYN Summary of this function goes here
%   Detailed explanation goes here
J=inertia_matrix_J(eta,I_B);
C = Coriolis(eta,eta_d);
etadd=inv(J)*(tau-C*eta_d);
end

