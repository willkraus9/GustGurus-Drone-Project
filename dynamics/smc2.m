function T = smc2(e,edot)
%SMC2 Summary of this function goes here
%   Detailed explanation goes here
alpha=1.5;
D=0.0;
g=9.81;
Az=0.35;
m=0.028
beta= (-alpha+Az/m)*edot+D/m-0.01-g;
T = -beta*sign(e);
end

