classdef EagleRealSystem < AbstractEagleSystem
% EAGLEREALSYSTEM describes the continuous system for the quadcopter, using 
% a state vector x. The system is fed with control signals u to steer it to 
% a desired state. The control signals are provided by the controller, and 
% are used to update the state vector.
% The output of the system is y, and is measured by the sensors mounted on 
% the quadcopter.
% EagleRealSystem is subclass of AbstractEagleSystem.
% 
% See also: AbstractEagleSystem

    properties (Access = private)
        p; % Quatcopter parameters
        sizeInput; % Number of system inputs
        sizeOutput; % Number of system outputs
        functionHandleQuatNormalize = @(x) x; % Function handler
        dynamics; % Continuous dynamics
    end % ... end of properties

    methods (Access = private)
        % No private methods are required
    end % ... end of private methods
    
    methods (Access = public)
        function self = EagleRealSystem(x0, p, sizeInput, sizeOutput, freq, dynamics, quatNormalize)
            % EAGLEREALSYSTEM is the constructor of EagleRealSystem, 
            % which creates the instances of the EagleRealSystem. 
            %
            % Syntax
            % obj = EagleRealSystem(x0,p, sizeInput,sizeOutput,freq, dynamics, quatNormalize) 
            %
            % Input arguments:
            % x0                Initial state
            % p                 Quatcopter parameters
            % sizeInput         Number of inputs
            % sizeOutput        Number of outputs
            % freq              Sampling frequency
            % dynamics          Continuous dynamics
            % quatNormalize     Function handler
            %
            % See also: AbstractEagleSystem
            self = self@AbstractEagleSystem(x0, freq); % Use constructor of superclass AbstractEagleSystem 
            self.p = p;
            self.sizeInput = sizeInput;
            self.sizeOutput = sizeOutput;
            self.dynamics = dynamics;
            if nargin == 7 % Check if function handler is provided
                self.functionHandleQuatNormalize = quatNormalize;
            end
        end
        
        function xdot = systemDynamics(self, x, u)
            % SYSTEMDYNAMICS describes the quadcopter dynamics using the 
            % state vector x. The plant is fed with control signals u to 
            % steer it to a desired state. 
            % The control signals are provided by the controller.
            %
            % Syntax
            % obj = EagleRealSystem(x0,p, sizeInput,sizeOutput,freq, dynamics, quatNormalize)   
            % xdot = systemDynamics(x,u)
            %
            % Input arguments:
            % x         Current state vector 
            % u         Control signals
            %
            % (Code copied from quat_dynamics) 
            xdot = self.dynamics(x,u,self.p);
        end
        
        function updateSystem(self, u)
            % UPDATESYSTEM is a function that updates the state vector x.
            % For that purpose it uses the control signals u, provided by 
            % the controller. 
            % The system is updated by integrating the differential 
            % equation that describes the system, as in systemDynamics.
            % 
            % Syntax
            % obj = EagleRealSystem(x0,p, sizeInput,sizeOutput,freq, dynamics, quatNormalize) 
            % updateSystem(u)
            %
            % Input arguments:
            % u         Control signals 
            %
            % See also: systemDynamics 
            
            % The controll signal stays the same during [0 t], the interval
            % over which the differential equation is integrated 
            samplingTime = 1/self.freq;
            [~, x] = ode45(@(t,x) self.systemDynamics(x, u), [0 samplingTime], self.getState()); % integrate differential equation from t=0 to t = sampling time  
            self.x = x(end, 1:end)';
            self.x(1:4) = self.functionHandleQuatNormalize(self.x(1:4)'); % Normalize the resulting quaternion
        end
        
        function y = getOutput(self, ~)
            % GETOUTPUT is a getter method that returns the current output 
            % of the system. 
            %
            % Syntax
            % obj = EagleRealSystem(x0,p, sizeInput,sizeOutput,freq, dynamics, quatNormalize)   
            % y = obj.getOutput()
            %
            % Input arguments:
            % None 
            y = self.x(1:self.sizeOutput);
        end
        
        function ny = getOutputSize(self)
            % GETOUTPUTSIZE is a getter method that returns the length of 
            % the output vector, which is stored internally.
            %
            % Syntax
            % obj = EagleRealSystem(x0,p, sizeInput,sizeOutput,freq, dynamics, quatNormalize) 
            % ny = getOutputSize()
            %
            % Input arguments:
            % None
            ny = self.sizeOutput;
        end
        
        function nu = getInputSize(self)
            % GETINPUTSIZE is a getter method that returns the length of 
            % the input vector, which is stored internally.
            %
            % Syntax
            % obj = EagleRealSystem(x0,p, sizeInput,sizeOutput,freq, dynamics, quatNormalize)
            % nu = getInputSize()
            %
            % Input arguments:
            % None
            nu = self.sizeInput;
        end
        
    end % ... end of public properties
end % ... end of class definition