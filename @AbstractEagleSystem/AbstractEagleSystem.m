classdef (Abstract) AbstractEagleSystem < handle
% ABSTRACTEAGLESYSTEM is the superclass of EagleLinearSystem and
% EagleRealSysteman. It is an abstract class that describes the 
% functionality common to both its subclasses, and also implements
% the possibility of adding noise using the NoiseGenerator. 
% Either way, the described system is fed with control signals to steer it 
% to a desired state. The control signals are provided by the controller, 
% and are used to update the state vector.
% 
% See also: EagleLinearSystem, EagleRealSystem, NoiseGenerator
    properties (Access = protected)
        x; % Current state vector
        noiseX NoiseGenerator % Noise generator object for X
        noiseY NoiseGenerator % Noise generator object for Y
        noiseU NoiseGenerator % Noise generator object for U
        freq = 238; % Sampling frequency
        functionHandleAlterQuatVec = @(x,y,z) x + y; % Function handler
    end % ... end of properties
    
    methods (Abstract)
        % Summary of abstract methods, used by EagleRealSystem and
        % EagleLinearSystem:
        % updateSystem updates the state vector
        % getOutput returns the current output of the system
        % getOutputSize returns the length of the output vector
        % getInputSize returns the length of the input vector
        %
        % See also: EagleRealSystem, EagleLinearSystem (more detailed 
        % documentation on the functions)
        updateSystem(self, x);
        getOutput(self, u);
        getOutputSize(self);
        getInputSize(self);
    end
    
    methods (Access = private)
        % '''''''''''''''''''''''''''''
        % Methods for class use only
        % '''''''''''''''''''''''''''''
    end % ... end of private methods
    
    methods (Access = public)
        % '''''''''''''''''''''''''''''
        % Public class methods go here
        % ? Constructor
        % ? Getters and Setters
        % ? Other methods
        % '''''''''''''''''''''''''''''
        function self = AbstractEagleSystem(x0, freq, alterQuatVec)
            % ABSTRACTEAGLESYSTEM is the constructor of 
            % AbstractEagleSystem, which creates the instances of 
            % AbstractEagleRealSystem. 
            %
            %
            % Input arguments:
            % x0            Initial state vector
            % freq          Sampling frequency
            % alterQuatVec  Function handler
            %
            % A detailed documentation on the function handler is
            % given in script alterQuatVec
            %
            % See also: alterQuatVec
            if nargin == 3 % Check if function handler are given to the constructor
                self.functionHandleAlterQuatVec = alterQuatVec;
            end
            self.x = x0;
            self.freq = freq;
        end
        
        function resetState(self, x0)
            % RESETSTATE is a function that resets the state vector to a
            % chosen input vector x0.
            %           
            % Syntax
            % resetState(x0)
            %
            % Input arguments:
            % x0    Initial state
            self.x = x0;
        end
        
        function state = getState(self)
            % GETSTATE is a getter method that returns the current state
            % vector of the system, which is stored internally.
            %           
            % Syntax
            % x = getState()
            state = self.x;
        end
        
        function nb = getStateSize(self)
            % GETNBSTATESIZE is a getter method that returns the number of 
            % states, which is equal to the length of the state vector of
            % the system (stored intenally).
            %           
            % Syntax
            % nb = getStateSize()
            nb = size(self.x, 1);
        end
        
        function setNoiseGenerator(self, noiseX, noiseY, noiseU)
            % SETNOISEGENERATOR is a setter method that sets the noise 
            % fields to a given value.
            %
            % Syntax
            % setNoiseGenerator(noiseX, noiseY, noiseU)
            %
            % Inputs
            % noiseX    noiseX value
            % noiseY    noiseY value
            % noiseZ    noiseZ value
            self.noiseX = noiseX;
            self.noiseY = noiseY;
            self.noiseU = noiseU;
        end
        
        function vec = getDisturbedU(self, u)
            % GETDISTURBEDU is a function that adds noise to a given vector
            % u, which represents the control signals.
            %
            % Syntax
            % vec = getDisturbedU(u)
            %
            % Inputs
            % u    vector to which add noise to
            if isempty(self.noiseU)
                vec = u;
            else
                vec = u + self.noiseU.getNoise(); 
            end
        end
        
        function vec = getDisturbedY(self, y)
            % GETDISTURBEDY is a function that adds noise to a given vector
            % y, which represents the output. 
            %
            % Syntax
            % vec = getDisturbedY(self, y)
            %
            % Inputs
            % y    vector to which add noise to
            if isempty(self.noiseX)
                vec = y;
            else
                noise = self.noiseY.getNoise(); % Get noise vector
                % If the output vector contains a quaternion (eg in
                % attitude control), noise must be added while still 
                % maintaining the normalisation of the quaternion. For this 
                % purpose, the function handler alterQuatVec is used. If Y 
                % does not contain a quaternion, the function handler just 
                % adds the noise vector to Y (see objectInitialisation and
                % default value).
                vec = self.functionHandleAlterQuatVec(y,noise,3); 
            end
        end
        
        function vec = getDisturbedX(self)
            % GETDISTURBEDX is a function that adds noise to the state 
            % vector. 
            %
            % Syntax
            % vec = getDisturbedX()
            %
            % Inputs
            % none
            if isempty(self.noiseX)
                vec = self.x;
            else
            noise = self.noiseX.getNoise(); % Get noise vector
            % If the state vector contains a quaternion (eg in attitude 
            % control), noise must be added while still maintaining the 
            % normalisation of the quaternion. For this purpose, the 
            % function handler alterQuatVec is used. If the state vector 
            % does not contain a quaternion, the function handler just adds
            % the noise vector noise to the state vector (see 
            % objectInitialisation and default value).
            vec = self.functionHandleAlterQuatVec(self.x,noise,30);
            end
        end
        
        function int = getSamplingFrequency(self)
            % GETSAMPLINGFREQUENCY is a getter method that returns the 
            % samplingfrequency, which is stored internally.
            %
            % Syntax
            % f = getSamplingFrequency()
            %
            % Inputs
            % none
            int = self.freq;
        end
        
    end % ... end of public properties
end % ... end of class definition