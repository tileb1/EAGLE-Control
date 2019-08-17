classdef EagleSimulator < handle
% EAGLESIMULATOR is a class that is used to simulate the systems for the 
% quadcopter. To perform this simulation, EagleSimulator is linked to the 
% system- (linear and real), the controller-, observer-, reference- and
% medianfilterclasses. For a given time and for a given reference signal, 
% the output y the system is simulated and plotted. The output is measured 
% by the sensors mounted on the quadcopter.
%
% See also: EagleControllerLQR, EagleControllerLQI, EagleObserver, 
% Reference, AbstractEagleSystem, EagleLinearSystem, EagleRealSystem, 
% NoiseGenerator, MedianFilter
    
    properties (Access = private)
        observer EagleObserver; % Object of linear observer-class
        controller AbstractEagleController = EagleControllerLQR(EagleLinearSystem([1; 1], discreteAttitudeSystem(quat_params, 1), 0), diag([1, 1, 1, 1, 1, 1, 1, 1, 1]), diag([1, 1, 1]), 0, 0, 0); % Object of controller-class
        system AbstractEagleSystem = EagleRealSystem([1; 0; 0; 0; 0; 0; 0; 0; 0], quat_params(), 3, 7, 238, @quat_dynamics, @quatnormalize); % Abstract system-class
        % The default value should never be used, but matlab gives a 
        % warning if no default value is given because it is abstract
        reference Reference; % Object of reference-class
        medianFilter MedianFilter; % Object of median filter-class
        functionHandleFormatState = @(x) x; % Function handler
        functionHandleFormatReference = @(x) x; % Function handler
        vectorString; % Plot information
    end % ... end of properties

    methods (Access = private)
        % No private methods are required
    end % ... end of private methods
    
    methods (Access = public)
        function self = EagleSimulator(observer, controller, system, reference, vec, filter, formatState, formatReference)
            % EAGLESIMULATOR is the constructor of EagleSimulator, 
            % which creates the instances of the EagleSimulator-class. The 
            % simulator is linked to the system- (linear and real), the 
            % controller-, observer-, reference- and medianfilterclasses.  
            % The system being simulated is an abstract system: it could 
            % either be the real one or the linear one. Input-parameter 
            % system determines which system is simulated: if an object of 
            % subclass EagleRealSystem is given, then the continuous real 
            % system is simulated. If on the other hand an object of 
            % subclass EagleLinearSystem is given, then the linearised 
            % system is simulated.
            %
            % Syntax
            % obj = EagleSimulator(observer, controller, system, reference, vec, filter, formatState, formatReference)
            %
            % Input arguments:
            % observer          Object of linear observer class 
            % controller        Object of proportional controller class
            % system            Object of system class 
            % reference         Object of reference class
            % vec               String array with plot information
            % filter            Object of median filter class
            % formatState       Function handler
            % formatReference   Function handler
            self.observer = observer;
            self.controller = controller;
            self.system = system;
            self.reference = reference;
            self.vectorString = vec;
            self.medianFilter = filter;
            if nargin == 8 % Check if function handlers are provided, else use default values
                self.functionHandleFormatState = formatState;
                self.functionHandleFormatReference = formatReference;
            end
        end
        
        function simulate(self, time, input)
            % SIMULATE is the function that performs the simulation. The
            % goal is to simulate and plot the output y of the system 
            % described by the field system, and thise for a given time 
            % and reference. The simulation-time is given by the 
            % inputparameter time, the reference-signal is randomly 
            % generated in the Reference-class, but can also be given to
            % the function through parameter input, which is assumed a
            % cache of reference signals.
            %
            % Syntax:
            % obj = EagleSimulator(observer, controller, system, reference, vec, filter, formatState, formatReference)
            % obj.simulate(time, input)
            %
            % Input
            % time    Simulation time
            % input   Cache of reference signals
            fs = self.system.getSamplingFrequency(); % Sampling frequency
            iteration = time * fs; % Number of iterations for a given time
            
            nu = self.system.getInputSize(); % Number of system inputs
            nx = self.system.getStateSize(); % Number of states
            nref = self.reference.getNbReferenceSignals(); % Number of reference signals
            len1 = size(self.functionHandleFormatState(zeros(nx, 1)),1); % Selection of states to be displayed
            len2 = size(self.functionHandleFormatReference(zeros(nref, 1)),1); % Corresponding reference signals
            assert(len2 == len1); % Check if number of displayed states equals number of reference signals
            
            u = zeros(nu, 1); % Initial control signal
            xCache = zeros(len1, iteration); % Cache of states
            refCache = zeros(len2, iteration); % Cache of reference signals
            
            if nargin == 3 % Check if reference signal is given as an input
                self.reference.setCache(input); 
            end
            
            for k = 1 : iteration
                    y = self.system.getOutput(u); % Get system output
                    y = self.system.getDisturbedY(y); % Disturbed output
                    self.observer.updateStateEstimate(y, u); % Update state estimate in the observer
                    xest = self.observer.getCurrentStateEstimate(); % Get the current state estimate from the observer
                    if nargin == 2 % Check if cache of reference signals are given as input
                        r = self.reference.getRandom(); % Get random reference signal
                    else
                       r = self.reference.get(); % Get reference from cache
                    end
                    u = self.controller.getControlSignal(xest, r, y); % Get control signals from the controller
                    u = self.system.getDisturbedU(u); % Add noise on control signals
                    self.system.updateSystem(u); % Update system based on control signals
                    state = self.system.getState(); % Get state vector
                    xCache(1:end, k) = self.functionHandleFormatState(state); % Add state estimate to cache
                    refCache(1:end, k) = self.functionHandleFormatReference(r); % Add current reference signal to cache
            end
            k = 1:iteration;
            colorVector = ['r', 'g', 'm', 'k', 'b', 'r', 'g', 'm', 'k', 'b'];
            figure('Name','Simulation: system output versus reference','NumberTitle','off')
            for j = 1:len1
                subplot(2,ceil(len1/2),j)
                plotState = plot(k/fs, xCache(j,:),'Linewidth',2); hold on
                plotReference = plot(k/fs, refCache(j,:),'--','Linewidth',1); hold on
                legendText = self.vectorString(j); 
                legend(legendText,legendText+" Reference")
                xlabel("Time [s]"), ylabel("State, reference");
                title('Output versus reference', 'fontsize',10);
                grid on;
                set(plotState, 'Color', colorVector(j))
                set(plotReference, 'Color', 'k')
            end
        end
        
        function setNoiseXYU(self, mean, cov)
            % SETNOISEXYU is a function that makes it possible to simulate
            % with noise.
            %
            % Syntax
            % obj = EagleSimulator(observer, controller, system, reference, vec, filter, formatState, formatReference)
            % simulatorLinear.setNoiseXYU(mean,cov) OR simulatorReal.setNoiseXYU(mean,cov)
            % obj.simulate(time, input)
            %
            % Inputs
            % mean    Mean value of the added nois
            % cov     Covariance of the noise
            %
            % See also: NoiseGenerator
            self.system.setNoiseGenerator(NoiseGenerator(self.system.getStateSize(), mean, cov), ...
                NoiseGenerator(self.system.getOutputSize(), mean, cov), NoiseGenerator(self.system.getInputSize(), mean, cov));
        end
               
    end % ... end of public properties
end % ... end of class definition