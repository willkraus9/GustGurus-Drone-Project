clear;

params = struct();

params.m=0.028;  %kg
params.d = 0.046;  % m arm length
params.r = 23.1348e-3;  % rotor radius %m
% params.Ix=1.395e-5;  % kg*m2
% params.Iy=1.436e-5;  % kg*m2
% params.Iz=2.173e-5;  % kg*m2
% params.I_B = diag([params.Ix, params.Iy, params.Iz]);  % kg*m2
params.Ix = 16.571710e-06;  % kg*m2
params.Iy = 16.655602e-06;  % kg*m2
params.Iz = 29.261652e-06;  % kg*m2
params.Ixy = 0.830806e-06;  % kg*m2
params.Ixz = 0.718277e-06;  % kg*m2
params.Iyz = 1.800197e-06;  % kg*m2

params.I_B = [params.Ix params.Ixy params.Ixz;
              params.Ixy params.Iy params.Iyz; 
              params.Ixz params.Iyz params.Iz]; 
params.kT=4e-5;  % Thrust coefficient
params.kD=2.4e-6;  % torque coefficient
params.Cd=7.9379e-12;  
params.Ct=3.1582e-10;  
params.Ctbar=params.d*params.Ct/sqrt(2);

params.TW=[params.Ct, params.Ct, params.Ct, params.Ct;
           -params.Ctbar, -params.Ctbar, params.Ctbar, params.Ctbar; 
           -params.Ctbar, params.Ctbar, params.Ctbar, -params.Ctbar; 
           -params.Cd, params.Cd, -params.Cd, params.Cd];

params.g=9.81;      
params.Ax=0.35;
params.Ay=params.Ax;
params.Az=params.Ax;
params.Tb= 9.81*0.028;


bus = Simulink.Bus.createObject(params);

% bus is now stored in the workspace as a Bus object.
% Save the bus object name
busName = bus.busName;