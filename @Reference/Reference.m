classdef Reference < handle
% REFERENCE is a class which creates a reference signal for the quadcopter
% system. This reference signal r represents the desired output of the 
% system. r used to determine the equilibrium state and control signals, 
% that are used to compute the approriate control action for the system to 
% reach a desired state with output equal to the reference.
% The reference class is used to simulate the system. 
%
% See also: EagleSimulator, EagleControllerLQR, EagleControllerLQI

    properties (Access = private)
        r; % Reference signal
        max; % Number of iterations with some fixed reference 
        counter; % Current iteration
        cache; % Cache of reference signals
        functionHandleRandom; % Function handler for random reference
        length; % Size of the reference r-vector
    end % ... end of properties

    methods (Access = private)
        function updateCache(self)
            % UPDATECACHE is a function that assigns the first reference
            % signal in cache to r, the current reference. This function
            % is used when simulating the system using a given reference 
            % cache (not randomly generated reference signals).
            self.r = self.cache(:, 1);
            self.cache = self.cache(:, 2:end); % Remove the current reference from the cache
        end
    end % ... end of private methods
    
    methods (Access = public)
        function self = Reference(counter, len, random)
            % REFERENCE is the constructor of Reference, which creates the 
            % instances of the Reference-class. 
            %
            % Syntax
            % obj = Reference(counter, len, random) 
            %
            % Input arguments:
            % counter    Number of iterations with fixed reference
            % len        Length of the reference vector
            % random     Function handler for random reference signal
            self.counter = counter;
            self.max = counter; 
            self.functionHandleRandom = random;
            % The reference signal is randomly initialised, using the 
            % function handler given by random.
            self.r = self.functionHandleRandom(); 
            self.length = len;
        end
        
        function ref = getRandom(self)
            % GETRANDOMSELF is a function that performs one iteration for 
            % the reference class, this means:
            % - If the counter has reached zero, set it back to its maximum
            %   value and create new reference.
            % - If the counter is not equal to zero, substract it by 1.
            % The current refence is given as the output of the function.
            %
            % When simulating, the reference signal thus remains constant
            % for a number of iterations equal to max. When the counter has 
            % reached zero, a new random reference is created.
            %
            % Syntax
            % obj = Reference(counter, len, random)
            % ref = getRandom()
            if self.counter ~= 0 % Check if counter is not equal to zero
                self.counter = self.counter - 1; % Substract 1
                ref = self.r; 
            else
                self.counter = self.max; % Set counter back to maximum 
                ref = self.functionHandleRandom(); % Create new random reference
                % The maximum angle for the reference signal is chosen as 15. Angles that are to back can cause the drone to crash.    
                self.r = ref;
            end
            
        end
        
        function nb = getNbReferenceSignals(self)
            % GETNBREFERENCESIGNALS is getter method that returns the 
            % the length of the reference signal vector, which is stored 
            % internally.
            %
            % Syntax
            % obj = Reference(counter, len, random) 
            % Nb = getNbReferenceSignals()
            nb = self.length;
        end
            
        function set(self,input)
            % SET is a setter-method that sets the reference signal r to a
            % given vector input. 
            % 
            % Syntax
            % obj = Reference(counter, len, random) 
            % set(input)
            %
            % Input
            % input    Reference signal
            self.r = input;
        end
        
        function ref = get(self)
            % GET is getter method that returns the current reference,
            % which is stored internally. If a reference cache is 
            % available, the reference is taken equal to the first element
            % in the cache, and the cache is updated.
            %
            % Syntax
            % obj = Reference(counter, len, random) 
            % ref = get()
            %
            % See also: updateCache
            if ~isempty(self.cache) % Check if reference cache is available
                self.updateCache();
            end
            ref = self.r;
        end
        
        function setCache(self, input)
            % SETCACHE is a setter method that sets cache equal to a given
            % input. This input is assumed a cache of reference signals.
            % 
            % Syntax
            % obj = Reference(counter, len, random) 
            % setCache(input)
            %
            % Input
            % input    Cache of reference signals
            self.cache = input;
        end
        
    end % ... end of public properties
end % ... end of class definition