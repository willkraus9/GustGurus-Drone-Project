clear;

params = struct();

params.m=0.33;  %kg
params.d= 39.73e-3;  % m arm length
params.r=23.1348;  % rotor radius %m
params.Ix=1.395e-5;  % kg*m2
params.Iy=1.436e-5;  % kg*m2
params.Iz=2.173e-5;  % kg*m2
params.I_B = diag([params.Ix, params.Iy, params.Iz]);  % kg*m2
params.kT=0.2025;  % Thrust coefficient
params.kD=0.11;  % torque coefficient
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


bus = Simulink.Bus.createObject(params);

% bus is now stored in the workspace as a Bus object.
% Save the bus object name
busName = bus.busName;