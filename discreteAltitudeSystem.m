function sysd = discreteAltitudeSystem(p,samplingFrequency)
% DISCRETEALTITUDESYSTEM is a function that returns the discrete-time 
% system that represents the quadcopter dynamics in altitude mode. The 
% system matrices are copied from eagle-control-slides and 
% ANC-quaternion-model, where the parameters are given is quat_params. 
% samplingFrequency is used to discretize the continuous-time system.
%
% Syntax
% sysd = discreteSystem(p,samplingFrequency)
%
% Inputs
% p                     Quadcopter parameters
% samplingFrequency     Sampling frequency for discretization

kt = 4*p.ct*p.rho*(p.Dp)^4/p.m;
A = [-p.k2, 0, 0; 0, 0, 1; 2*p.nh*kt, 0, 0];
B = [p.k1*p.k2;0 ;0];
C = [0, 1, 0];
D = 0;
sys = ss(A,B,C,D); % Continuous-time system
sysd = c2d(sys, 1/samplingFrequency, 'zoh'); % Discrete-time system

