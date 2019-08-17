classdef (Abstract) AbstractEagleController < handle
% ABSTRACTEAGLECONTROLLER is the superclass of EagleControllerLQI and
% EagleControllerLQR. It is an abstract class that describes the 
% functionality common to both its subclasses. Either way, the controller 
% has two main functions: stabilize the system and steer it to a desired 
% state with output equal to a certain reference. For the latter, the 
% controller generates control signals u wich are fed to the plant. 
% To generate these control signals, the controller disposes of 
% - the state estimate xest, provided by the observer.
% - the reference signal r, that represents the desired output.
%
% See also: EagleControllerLQI, EagleControllerLQR
    
    % Private properties (not accessible from outside)
    properties (Access = protected)
        linearSystem EagleLinearSystem; % Discrete-time linear system, object of the EagleLinearSystem class
        G;  % Matrix for determining equilibrium state
        nx; % Number of states
        nu; % Number of control signals
        ny; % Number of outputs
        functionHandleCut = @(x) x; % Function handler
        functionHandleDiff = @(x, y) x - y; % Function handler
        functionHandleFormatVector = @(x, y) x; % Function handler
    end % ... end of properties
    
    methods (Abstract)
        % Summary of abstract methods, used by EagleControllerLQI and
        % EagleControllerLQR.
        % getControlSignal returns the current control signal
        %
        % See also: EagleControllerLQI, EagleControllerLQR
        u = getControlSignal(self, xest, r);
    end
    
    methods (Access = private)
        % '''''''''''''''''''''''''''''
        % Methods for class use only
        % '''''''''''''''''''''''''''''
        function initialiseSystemMatrices(self, linearSys)
            % INITIALISESYSTEMMATRICES initialises attributes linearSystem, 
            % nx, nu and ny of EagleController using the discrete-time 
            % linear system linearSys.
            %
            % Inputs
            % linearSys    discrete-time linear system
            %
            % See also: discreteSystem
            self.linearSystem = linearSys; % Discrete-time linear system
            self.nx = length(self.linearSystem.getA()); % number of states
            self.nu = size(self.linearSystem.getB(), 2); % number of inputs
            self.ny = size(self.linearSystem.getC(), 1); % number of outputs
        end % end function
        
        function initialiseG(self)
            % INITIALISEG initialises the matrix G of EagleController. This 
            % matrix is used to determine the equilibrium state (given a
            % certain reference) and control signals, which are necessary 
            % for reference tracking. 
            %
            % Inputs
            % None
            Ad = self.linearSystem.getA(); 
            Bd = self.linearSystem.getB();
            Cd = self.linearSystem.getC(); 
            Dd = self.linearSystem.getD();
            W = [Ad-eye(self.nx),Bd;Cd,Dd]; 
            self.G = W\[zeros(self.nx,self.ny);eye(self.ny)]; 
        end
    end % ... end of private methods
    
    methods (Access = public)
        % '''''''''''''''''''''''''''''
        % Public class methods go here
        % ? Constructor
        % ? Getters and Setters
        % ? Other methods
        % '''''''''''''''''''''''''''''
        function self = AbstractEagleController(linearSys, cut, diff, format)
            % ABSTRACTEAGLECONTROLLER is the constructor of 
            % AbstractEagleController, which creates the instances of the 
            % AbstractEagleController-class. 
            % 
            %
            % Input arguments:
            % linearSys    Discrete-time linear system 
            % cut          Function handler
            % diff         Function handler
            % format       Function handler
            % 
            % A detailed documentation on the function handlers is
            % given in scripts cut, quatDiff, formatVector 
            % 
            % See also: initialiseSystemMatrices, initialiseG, cut, 
            % quatDiff, formatVector 
            initialiseSystemMatrices(self, linearSys);
            initialiseG(self);
            if nargin == 4 % Check if function handlers are given to the constructor
                self.functionHandleCut = cut;
                self.functionHandleDiff = diff;
                self.functionHandleFormatVector = format;
            end
        end
        
        function G = getG(self)
            % GETG is a getter method that returns the matrix G used to 
            % determine the systems equilibrium point. G is stored 
            % internally, therefore no inputs are required.
            %
            % Syntax
            % G = obj.getG()
            %
            % Input arguments:
            % None
            %
            % See also: initialiseG
            G = self.G;
        end
        
        function sys = getLinearSystem(self)
            % GETLINEARSYSTEM is a getter method that returns the 
            % linear system that the controller acts on. This linear system
            % is stored internally, therefore no inputs are required.
            %
            % Syntax
            % sys = obj.getLinearSystem()
            %
            % Input arguments:
            % None
            sys = self.linearSystem;
        end
    end % ... end of public properties
end % ... end of class definition