function sysd = discreteAttitudeSystem(p, samplingFrequency)
% DISCRETEATTITUDESYSTEM is a function that returns the discrete-time 
% linear system for the attitude controller. The system matrices are copied 
% from eagle-control-slides and ANC-quaternion-model, where the parameters 
% are given is quat_params. samplingFrequency is used to discretize the 
% continuous-time system.
%
% Syntax
% sysd = discreteAttitudeSystem(p, samplingFrequency)
%
% Input
% p                    Quatcopter parameters
% samplingFrequency    Sampling frequency
    A1 = [zeros(3,3),diag([0.5 0.5 0.5]),zeros(3,3)];
    A2 = [zeros(3,3),zeros(3,3),p.gamma_n];
    A3 = [zeros(3,3),zeros(3,3),-p.k2*eye(3)];
    A = [A1;A2;A3];
    B = [zeros(3,3) ; p.gamma_u ; p.k1*p.k2*eye(3)];
    C = [eye(6), zeros(6,3)];
    D = zeros(6,3);
    sys = ss(A, B, C, D); % Continuous-time system
    sysd = c2d(sys, 1/samplingFrequency,'zoh'); % Discrete-time system
end
