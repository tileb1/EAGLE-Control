classdef EagleObserver < handle
% EAGLEOBSERVER is a class which implements the linear state observer for 
% the quadcopter. Since one cannot measure the exact state of the 
% quadcopter, the goal of the observer is to determine an estimate of this 
% state vector. To generate the estimated state, the observer disposes of 
% - the output y of the system, which is measured by the drone's sensors
% (IMU, sonar...).
% - the control signals u, which are given to the plant to 
%   steer it to a desired state. 
%   The control signals are provided by the controller.
%
% See also: AbstractEagleController, EagleControllerLQI, EagleControllerLQR
    
    % Private properties (not accessible from outside)
    properties (Access = private)
        linearSystem; % Discrete-time linear system
        xest; % estimated state vector
        L; % Gain of the linear observer
        functionHandleCut = @(x) x; % Function handler
        functionHandleDiff = @(x, y) x - y; % Function handler
        functionHandleFormatVector = @(x, y) x; % Function handler
    end % ... end of properties
    
    methods (Access = private)
        function initialiseL(self, covarDynamics, covarSensors)
            % INITIALISEL initialises attribute L of EagleObserver. This 
            % matrix represents the gain of the linear observer, and is 
            % used to determine the state estimate. 
            %
            % Inputs
            % covarDynamics    Covariance matrix for input and model
            % covarSensors     Covariance matrix for output
            Ad = self.linearSystem.getA(); 
            Bd = self.linearSystem.getB();
            Cd = self.linearSystem.getC(); 
            % Dd = self.linearSystem.getD();
            nx = size(Bd,1);
            G0 = [Bd eye(nx)];
            kalmanSystem = ss(Ad, [Bd G0], Cd, 0, -1); % Discrete-time system used do design the kalman filter 
            [~, Lobserver] = kalman(kalmanSystem , covarDynamics , covarSensors , 0); % Linear observer gain
            self.L = -Lobserver;
        end 
    end % ... end of private methods
    
    methods ( Access = public )
        % '''''''''''''''''''''''''''''
        % Public class methods go here
        % ? Constructor
        % ? Getters and Setters
        % ? Other methods
        % '''''''''''''''''''''''''''''
        function self = EagleObserver(linearSys, xest0, covarDynamics, covarSensors, cut, diff, format)
            % EAGLEOBSERVER is the constructor of EagleObserver, 
            % which creates the instances of the EagleController-class. 
            %
            % Syntax
            % obj = EagleObserver(linearSys,xest0, covarDynamics, covarSensors, @cut, @quatDiff, @formatVector) 
            %
            % Input arguments:
            % linearSys        Discrete-time linear system 
            % xest0            Initial state estimate 
            % covarDynamics    Covariance for w
            % covarSensors     Covariance for v
            % cut              Function handler, see cut
            % diff             Function handler, see quatDiff
            % format           Function handler, see formatVector
            %
            % See also: initialiseL, cut, quatDiff, formatVector
            self.linearSystem = linearSys;
            initialiseL(self, covarDynamics, covarSensors);
            self.xest = xest0;
            if nargin == 7 % Check if function handlers are provided, else use default value
                self.functionHandleCut = cut;
                self.functionHandleDiff = diff;
                self.functionHandleFormatVector = format;
            end
        end
        
        function x = getCurrentStateEstimate(self)
            % GETCURRENTSTATEESTIMATE returns the current state estimate    
            % x, which is provided by the observer.
            % x is stored internally, therefore no inputs are required.
            %
            % Syntax
            % obj = EagleObserver(linearSys,xest0, covarDynamics, covarSensors, @cut, @quatDiff, @formatVector)  
            % x = obj.getCurrentStateEstimate()
            %
            % Input arguments:
            % None
            x = self.xest ;
        end
        
        function yest = getCurrentOutputEstimate(self)
            % GETCURRENTOUTPUTESTIMATE returns the estimated output 
            % y of the plant.
            % The estimated output is calculated using the estimated state
            % vector xest and the system matrix Cd, which are both stored 
            % internally, therefore no inputs are required.
            %
            % Syntax
            % obj = EagleObserver(linearSys,xest0, covarDynamics, covarSensors, @cut, @quatDiff, @formatVector)  
            % yest = obj.getCurrentOutputEstimate()
            %
            % Input arguments:
            % None
            % 
            % See also: getCurrentStateEstimate
            yest = self.linearSystem.getC() * self.functionHandleCut(self.xest); % In case of attitude system, yest is a6 vector output, thus q0 not included
        end
        
        function updateStateEstimate(self, y, u)
            % UPDATESTATEESTIMATE updates the state estimate x of the 
            % system. u is the vector of control signals, provided by the 
            % controller. y is the output of the system, as measured by the 
            % IMU, sonar, ... .
            %
            % Syntax
            % obj = EagleObserver(linearSys,xest0, covarDynamics, covarSensors, @cut, @quatDiff, @formatVector) 
            % obj.updateStateEstimate(y,u)
            %
            % Input arguments:
            % y    Sensor measurements of the output of the system
            % u    Control signals
            %
            % See also: EagleControllerLQR, EagleControllerLQI, 
            % getCurrentOutputEstimate, initialiseL
            yest = self.getCurrentOutputEstimate(); % Output estimate
            % If updateStateEstimate is used for the attitude control
            % system, a substraction of quaternions should be performed
            % using the Hamilton product. functionHandleFormatVector
            % and functionHandleDiff perform this task. In this case, only
            % the 9-component state vector should be used for computations.
            % functionHandleFormatVector is used for this purpose.
            yError = self.functionHandleCut(self.functionHandleDiff(self.functionHandleFormatVector(yest, y), y)); % Error on output estimate
            xestNext = self.linearSystem.getA() * self.functionHandleCut(self.xest) + ...
                self.L * yError + self.linearSystem.getB() * u;
            self.xest = self.functionHandleFormatVector(xestNext, y);
        end
        
        function observer = getL(self)
            % GETL returns the matrix L as calculated by the kalman filter.
            % L is stored internally, therefore no inputs are required.
            %
            % Syntax
            % obj = EagleObserver(linearSys,xest0, covarDynamics, covarSensors, @cut, @quatDiff, @formatVector)  
            % L = obj.getL()
            %
            % Input arguments:
            % None
            %
            % See also: initialiseL
            observer = self.L;
        end
    end % ... end of public properties
end % ... end of class definition