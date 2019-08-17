classdef NoiseGenerator
    % NOISEGENERATOR is class which creates noise on a vector that is given
    % to it. The generated noise is a vector, equal in size to the vector 
    % given to it. The noise is normal distributed with a certain mean 
    % and variance. The class can be used to add noise on the state vector,
    % control signals and measurements during simulations.
    %
    % See also: AbstractEagleSystem
    properties (Access = private)
        covar; % Covariance matrix 
        mean; % Mean value
    end % ... end of properties
    
    methods (Access = public)
        % '''''''''''''''''''''''''''''
        % Public class methods go here
        % ? Constructor
        % ? Getters and Setters
        % ? Other methods
        % '''''''''''''''''''''''''''''
        function self = NoiseGenerator(N, mean, covar)
            % NOISEGENERATOR is the constructor of NoiseGenerator, which 
            % creates the instances of the NoiseGenerator. 
            %
            % Syntax
            % obj = NoiseGenerator(N,mean,covar) 
            %
            % Input arguments:
            % N        Length of the vector
            % mean     Mean value
            % covar    Covariance matrix
            if isscalar(mean) % If mean is a scalar, the mean value of all the N components of the noise vector is equal to mean 
                vec = zeros(N, 1); % Create vector with proper length
                vec(:) = mean; % Change every component to mean
                self.mean = vec;
            else % Mean is a vector
                self.mean = mean; 
            end
            if isscalar(covar)% If covar is a scalar, the N diagonal elemnts of the covariance matrix are equal to covar 
                matrix = zeros(N, N); % Create matrix with proper dimensions 
                matrix(1:N+1:end) = covar;
            else % covar is a matrix
                matrix = covar;
            end
            self.covar = matrix;
        end
        
        function noise = getNoise(self)
            % GETNOISE is a getter method that returns the noise vector, 
            % which is stored internally. Each component of the noise 
            % vector has a normal distribution with a mean value mean 
            % (stored internally), the noise vector is characterised by a 
            % covariance matrix internally stored in covar. 
            %
            % Syntax
            % obj = NoiseGenerator(N,mean,covar)
            % noise = getNoise()
            noise = mvnrnd(self.mean, self.covar)'; 
        end
    end % ... end of public properties
end % ... end of class definition