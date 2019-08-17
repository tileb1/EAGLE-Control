classdef EagleLinearSystem < AbstractEagleSystem
% EAGLELINEARSYSTEM describes the linearised system for the quadcopter. 
% The linearisation is performed based on the quatcopter dynamics as 
% described in the system of EagleRealSystem. The system is always 
% linearised around its equilibrium point, next discretised using zero-
% order-hold with sampling frequency f = 238Hz or f = 10Hz depending on the 
% system. The system is fed with control signals to steer it to a desired 
% state. The control signals are provided by the controller, and are used 
% to update the state vector.
% EagleLinearSystem is subclass of AbstractEagleSystem.
% 
% See also: AbstractEagleSystem, EagleRealSystem
    properties (Access = private)
        Ad; % Discrete-time system matrix Ad
        Bd; % Discrete-time system matrix Bd
        Cd; % Discrete-time system matrix Cd
        Dd; % Discrete-time system matrix Dd
        functionHandle = 0; % Function handler
    end % ... end of properties

    methods (Access = private)
        function initialiseSystemMatrices(self, sysd)
            % INITIALISESYSTEMMATRICES initialises attributes Ad, Bd, Cd, 
            % Dd of EagleLinearSystem and the freq-field of 
            % AbstractEagleSystem using the discrete-time system sysd.
            %
            % Inputs
            % sysd    discrete-time system
            %
            % See also: discreteSystem
            [self.Ad, self.Bd, self.Cd, self.Dd] = ssdata(sysd);
            self.freq = 1/sysd.ts; % initialise freq field of AbstractEagleSystem
        end % end function
    end % ... end of private methods
    
    methods (Access = public)
        function self = EagleLinearSystem(x0, sysd, freq, functionHandle)
            % EAGLELINEARSYSTEM is the constructor of EagleLinearSystem, 
            % which creates the instances of the EagleLinearSystem. 
            %
            % Syntax
            % obj = EagleLinearSystem(x0,sysd,freq,functionHandle) 
            %
            % Input arguments:
            % x0                Initial state
            % sysd              Discrete-time system (Quadcopter dynamics)
            % freq              Sampling frequency
            % functionHandle    Function handler
            %
            % See also: AbstractEagleSystem
            self = self@AbstractEagleSystem(x0, freq); % Use constructor of superclass AbstractEagleSystem
            initialiseSystemMatrices(self, sysd);
            if nargin == 4 % If function handler is provided, initialise it, else use default value
                self.functionHandle = functionHandle;
            end
        end
        
        function updateSystem(self, u)
            % UPDATESYSTEM is a function that updates the state vector 
            % which is stored as a field of AbstractEagleSystem. However, 
            % noise is added by the NoiseGenerator.
            % To update the state vector, the function uses the control 
            % signals u provided by the controller.   
            % 
            % Syntax
            % obj = EagleLinearSystem(x0,sysd,freq,functionHandle)  
            % obj.updateSystem(u)
            %
            % Input arguments:
            % u      Control signals 
            %
            % See also: NoiseGenerator 
            state = self.x;
            state = self.Ad * state + self.Bd * u; % Next state
            self.x = state;
            % The linearised model for attitude control does not provide 
            % q0, although this component is important while simulating the 
            % system. Therefore, we apply a little trick. We initialise 
            % the system matrices as defined in discreteSystem, however we
            % add a zero row and zero column in these matrices (see 
            % objectInitialisation). By checking the zero row, zero column 
            % condition, updateSystem can verify that the attitude control 
            % "trick" system is being updated, and that an extra 
            % calculation is necessary to determine q0. However, any other 
            % linear system is updated as usual (xDot = A*x+B*y). This is
            % done using the function handler.
            % See also: objectInitialisation
            if ~isequal(self.functionHandle, 0)
                self.functionHandle(self);
            end

        end
        
        function y = getOutput(self, u)
            % GETOUTPUT is a getter method that returns the current output 
            % vector of the system. The output is calculated using
            % the state vector (internally stored) and the input control 
            % signals.
            %
            % Syntax
            % obj = EagleLinearSystem(x0,sysd,freq,functionHandle) 
            % y = obj.getOutput(u)
            %
            % Input arguments:
            % u    Control signals
            y = self.Cd * self.x + self.Dd * u;
        end
        
        function a = getA(self)
            % GETA is a getter method that returns the A matrix of the 
            % linearised discrete-time system, which is stored internally.
            %
            % Syntax
            % obj = EagleLinearSystem(x0,sysd,freq,functionHandle)  
            % A = obj.getA()
            %
            % Input arguments:
            % None 
            a = self.Ad;
        end
        
        function a = getB(self)
            % GETB is a getter method that returns the B matrix of the 
            % linearised discrete-time system, which is stored internally.
            %
            % Syntax
            % obj = EagleLinearSystem(x0,sysd,freq,functionHandle)
            % B = obj.getB()
            %
            % Input arguments:
            % None 
            a = self.Bd;
        end
        
        function a = getC(self)
            % GETC is a getter method that returns the C matrix of the 
            % linearised discrete-time system, which is stored internally.
            %
            % Syntax
            % obj = EagleLinearSystem(x0,sysd,freq,functionHandle)
            % C = obj.getC()
            %
            % Input arguments:
            % None 
            a = self.Cd;
        end
        
        function a = getD(self)
            % GETD is a getter method that returns the D matrix of the 
            % linearised discrete-time system, which is stored internally.
            %
            % Syntax
            % obj = EagleLinearSystem(x0,sysd,freq,functionHandle) 
            % D = obj.getD()
            %
            % Input arguments:
            % None 
            a = self.Dd;
        end
        
        function a = getFunctionHandle(self)
            % GETFUNCTIONHANDLE is a getter method that returns the
            % function handler currently assigned to EagleLinearSystem.
            %
            % Syntax
            % obj = EagleLinearSystem(x0,sysd,freq,functionHandle) 
            % a = getFunctionHandle()
            %
            % Input arguments:
            % None
            a = self.functionHandle;
        end
        
        function ny = getOutputSize(self)
            % GETOUTPUTSIZE is a getter method that returns the length of 
            % the output vector, which equals the rownumber of the C-matrix
            % of the linear system.
            %
            % Syntax
            % obj = EagleLinearSystem(x0,sysd,freq,functionHandle) 
            % ny = getOutputSize()
            %
            % Input arguments:
            % None
            ny = size(self.Cd,1);
        end
        
        function nu = getInputSize(self)
            % GETINPUTSIZE is a getter method that returns the length of 
            % the input vector, which equals the columnnumber of the 
            % B-matrix of the linear system.
            %
            % Syntax
            % obj = EagleLinearSystem(x0,sysd,freq,functionHandle) 
            % nu = getInputSize()
            %
            % Input arguments:
            % None
            nu = size(self.Bd,2);
        end
        
    end % ... end of public properties
end % ... end of class definition