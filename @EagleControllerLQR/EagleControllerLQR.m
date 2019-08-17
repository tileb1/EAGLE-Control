classdef EagleControllerLQR < AbstractEagleController
% EAGLECONTROLLERLQR is a class which implements the LQR-control action
% for the quadcopter, and is a subclass of AbstractEagleController. The 
% controller has two main functions: stabilize the system and steer the 
% system into a desired state with output equal to a certain reference. For 
% the latter, the controller generates control signals u which are given to 
% the plant. To generate the control signals, the 
% controller disposes of 
% - the state estimate xest, which is provided by the observer.
% - the reference signal r, which represents the desired output.
% The controller uses proportional action. 
%
% See also: AbstractEagleController
    properties (Access = private)
        K; % Controller matrix
    end % ... end of properties

    methods (Access = private)
        function initialiseK(self, Q, R)
            % INITIALISEK initialises attribute K of EagleController. This 
            % matrix represents the proportional gain of the controller,
            % and is used to determine the control signals.
            %
            % Inputs
            % Q    LQR-matrix Q
            % R    LQR-matrix R
            Ad = self.linearSystem.getA(); 
            Bd = self.linearSystem.getB(); 
            if rank (ctrb (Ad,Bd),10^-6)~= self.nx % First check controllability
                error ('System not controllable'); 
            end
            Kcontroller = -dlqr(Ad,Bd,Q,R);
            assert ( all (abs( eig(Ad+Bd*Kcontroller )) <= 1-10^-7) ,'K is not stabilising '); % Assert that Kcontroller is a stabilising gain
            self.K = Kcontroller;
        end
    end % ... end of private methods
    
    methods (Access = public)
        function self = EagleControllerLQR(linearSys, Q, R, cut, diff, format)
            % EAGLECONTROLLERLQR is the constructor of EagleControllerLQR, 
            % which creates the instances of the EagleControllerLQR-class. 
            %
            % Syntax
            % obj = EagleControllerLQR(linearSys, Q, R, cut, diff, format) 
            %
            % Input arguments:
            % linearSys    Discrete-time linear system
            % Q            LQR-matrix Q
            % R            LQR-matrix R
            % cut          Function handler (select vector components) 
            % diff         Function handler (quaternion difference)
            % format       Function handler (scalar quaternion part)
            %
            % See also: cut, quatDiff, formatVector,
            % AbstractEagleController, initialiseK
            self = self@AbstractEagleController(linearSys, cut, diff, format); % Use constructor of superclass AbstractEagleSystem
            initialiseK(self, Q, R);
        end
        
        function u = getControlSignal(self, xest, r, ~)
            % GETCONTROLSIGNAL is a function that returns the control
            % signals, that should steer the system to a desired state with
            % output equal to the reference. To do this, the controller
            % disposes over the state estimate (provided by the observer)
            % and the reference signal.
            %
            % Syntax
            % obj = EagleControllerLQR(linearSys, Q, R, cut, diff, format)
            % u = obj.getControlSignal(self, xest, r, ~)
            %
            % Input arguments
            % xest     State estimate
            % r        Reference signal
            xue = self.G * self.functionHandleCut(r); % Equilibrium point xue = [xe, ue]', xe equilibrium state, ue equilibrium input 
            xe = self.functionHandleFormatVector(xue(1:self.nx), r); % Equilibrium state
            % functionHandleFormatVector calculates the scalar part of the 
            % position quaternion q0 (if getControlSignal acts on a state 
            % vector containing a quaterion, the proper function handler,
            % that is formatVector, will be given to the constructor). If 
            % no such operation is necessary (e.g. for the altitude hold
            % system), functionHandleFormatVector will perform no
            % operation.
            ue = xue(self.nx + 1, end); % Equilibrium control signals
            u = ue + self.K * self.functionHandleCut(self.functionHandleDiff(xest, xe)); % Control signals
        end
        
        function K = getK(self)
            % GETK is a getter method that returns the controller matrix K, 
            % which is stored internally.
            %
            % Syntax
            % obj = EagleControllerLQR(linearSys, Q, R, cut, diff, format)
            % K = obj.getK(self)
            %
            % Input arguments
            % none
            K = self.K;
        end
        
    end % ... end of public properties
end % ... end of class definition