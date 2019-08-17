function sysd = discreteLoiterSystem(p,samplingFrequency, lambda)
% DISCRELOITERSYSTEM is a function that returns the discrete-time 
% linear system for loiter mode. The system matrices are copied 
% from eagle-control-slides, where the parameters 
% are given is quat_params. samplingFrequency is used to discretize the 
% continuous-time system. lambda is a system parameter that is determined 
% using data.
%
% Syntax
% sysd = discreteLoiterSystem(quat_params, samplingFrequency, lambda)
%
% Input
% p                    Quatcopter parameters
% samplingFrequency    Sampling frequency
% lambda               System parameter
    A1 = [-lambda,0,0,0,0,0; 0,-lambda,0,0,0,0];
    A2 = [0,0,0,0,1,0;0,0,0,0,0,1];
    A3 = [0,2*p.g,0,0,0,0;-2*p.g,0,0,0,0,0];
    A = [A1;A2;A3];
    B = [lambda*eye(2);zeros(4,2)];
    C = [eye(4),zeros(4,2)];
    D = zeros(4,2);
    sys = ss(A, B, C, D); % Continuous-time system
    sysd = c2d(sys, 1/samplingFrequency,'zoh'); % Discrete-time system
end
