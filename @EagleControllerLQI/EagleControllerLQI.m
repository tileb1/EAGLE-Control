classdef EagleControllerLQI < AbstractEagleController
% EAGLECONTROLLERLQI is a class which implements the LQI-control action
% for the quadcopter, and is a subclass of AbstractEagleController. The 
% controller has two main functions: stabilize the system and steer the 
% system into a desired state with output equal to a certain reference. For 
% the latter, the controller generates control signals u which are given to 
% the plant. To generate the control signals, the 
% controller disposes of 
% - the state estimate xest, which is provided by the observer.
% - the reference signal r, which represents the desired output.
% The controller also uses integral action. 
%
% See also: AbstractEagleController
    properties (Access = private)
        Kx; % Controller matrix for state vector
        Kz; % Controller matrix for integrated error
        zPast; % Previous integrated error
    end % ... end of properties

    methods (Access = private)
        function initialiseK(self, Q, R)
            % INITIALISEK initialises the attributes of EagleControllerLQI. 
            % Q and R are matrices used ro determine the control matrix K,
            % which consists of Kx and Kz.
            %
            % Inputs
            % Q    LQI-matrix Q
            % R    LQI-matrix R
            Ad = self.linearSystem.getA(); % Discrete-time system matrix A 
            Bd = self.linearSystem.getB(); % Discrete-time system matrix B
            Cd = self.linearSystem.getC(); % Discrete-time system matrix C
            Dd = self.linearSystem.getD(); % Discrete-time system matrix D
            samplingFreq = self.linearSystem.getSamplingFrequency(); % Samplinf frequency
            if rank (ctrb (Ad,Bd),10^-6)~= self.nx % First check controllability
                error ('System not controllable'); 
            end
            Kcontroller = -lqi(ss(Ad, Bd, Cd, Dd, 1/samplingFreq),Q,R); % Full controller matrix, consisting of Kx and Kz
            self.Kx = Kcontroller(1:self.nu,1:self.nx); % Kontroller matrix Kx for states
            self.Kz = Kcontroller(1:self.nu,self.nx+1:end); % Kontroller matrix Kz for integrated error
        end
    end % ... end of private methods
    
    methods (Access = public)
        function self = EagleControllerLQI(linearSys, Q, R, cut, diff, format)
            % EAGLECONTROLLERLQI is the constructor of EagleControllerLQI, 
            % which creates the instances of the EagleControllerLQI-class. 
            %
            % Syntax
            % obj = EagleControllerLQI(linearSys, Q, R, cut, diff, format) 
            %
            % Input arguments:
            % linearSys    Discrete-time linear system
            % Q            LQI-matrix Q
            % R            LQI-matrix R
            % cut          Function handler (select vector components) 
            % diff         Function handler (quaternion difference)
            % format       Function handler (scalar quaternion part)
            %
            % See also: cut, quatDiff, formatVector,
            % AbstractEagleController, initialiseK
            if nargin == 3
            % If no function handlers are given, cut, diff and format are
            % initialised:
                cut = @(x) x; % No operation
                diff = @(x, y) x - y; % Vector substraction
                format = @(x, y) x; % No operation
            end
            self = self@AbstractEagleController(linearSys, cut, diff, format); % Use constructor of superclass AbstractEagleSystem
            self.zPast = zeros(self.linearSystem.getOutputSize(), 1); % z is the integrated error (= reference - output), thus has the same dimensions as the output
            initialiseK(self, Q, R);
        end
        
        function u = getControlSignal(self, xest, r, y)
            % GETCONTROLSIGNAL is a function that returns the control
            % signals, that should steer the system to a desired state with
            % output equal to the reference. To do this, the controller
            % disposes over the state estimate (provided by the observer),
            % the reference signal and output vector.
            %
            % Syntax
            % obj = EagleControllerLQI(linearSys, Q, R, cut, diff, format)
            % u = obj.getControlSignal(self, xest, r, y)
            %
            % Input arguments
            % xest     State estimate
            % r        Reference signal
            % y        Output vector 
            xue = self.G * self.functionHandleCut(r); % Equilibrium point xue = [xe, ue]'
            xe = self.functionHandleFormatVector(xue(1:self.nx), r); % Equilibrium state
            % functionHandleFormatVector calculates the scalar part of the 
            % position quaternion q0 (if getControlSignal acts on a state 
            % vector containing a quaterion, the proper function handler,
            % that is formatVector, will be given to the constructor). If 
            % no such operation is necessary (e.g. for the altitude hold
            % system), functionHandleFormatVector will perform no
            % operation.
            ue = xue(self.nx + 1, end); % Equilibrium control signals
            uLQI = self.Kz * self.zPast; % Integral action
            self.zPast = self.zPast + 1/self.linearSystem.getSamplingFrequency() * (r - y); % Integrated error (in discrete-time)
            u = ue + self.Kx * self.functionHandleCut(self.functionHandleDiff(xest, xe)) + uLQI; % Control signals
        end
        
        function Kx = getKx(self)
            % GETKX is a getter method that returns Kx, the part of the 
            % controller matrix K that acts on the states. Kx is stored 
            % internally.
            %
            % Syntax
            % obj = EagleControllerLQI(linearSys, Q, R, cut, diff, format)
            % Kx = obj.getKx(self)
            %
            % Input arguments
            % none
            Kx = self.Kx;
        end
        
        function Kz = getKz(self)
            % GETKZ is a getter method that returns Kz, the part of the 
            % controller matrix K that acts on the integrated error. Kz is 
            % stored internally.
            %
            % Syntax
            % obj = EagleControllerLQI(linearSys, Q, R, cut, diff, format)
            % Kx = obj.getKz(self)
            %
            % Input arguments
            % none
            Kz = self.Kz;
        end
        
    end % ... end of public properties
end % ... end of class definition