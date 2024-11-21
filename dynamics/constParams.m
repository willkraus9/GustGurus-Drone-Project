clear;

m=0.33;  %kg
d= 39.73e-3;  % m arm length
r=23.1348;  % rotor radius %m
Ix=1.395e-5;  % kg*m2
Iy=1.436e-5;  % kg*m2
Iz=2.173e-5;  % kg*m2
I_B = diag([Ix,Iy,Iz]);  % kg*m2
kT=0.2025;  % Thrust coefficient
kD=0.11;  % torque coefficient
Cd=7.9379e-12;  
Ct=3.1582e-10;  
Ctbar=d*Ct/sqrt(2);
TW=[Ct, Ct, Ct, Ct; -Ctbar, -Ctbar, Ctbar, Ctbar; -Ctbar, Ctbar, Ctbar, -Ctbar; -Cd, Cd, -Cd, Cd];
g=9.81;
Ax=0.35;
Ay=Ax;
Az=Ax;