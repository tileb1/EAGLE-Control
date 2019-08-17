classdef MedianFilter < handle
% MEDIANFILTER is a class which implements the median filter for 
% the quatcopter. The median filter is used to reject outliers, present in 
% sensor measurements. The median filter solves this problem by
% returning a filtered measurement that is equal to the median of some 
% number of past measurements.
    properties (Access = private)
        m; % Number of sonar measurements
        data; % Past m sonar measurements
        xvec;
    end % ... end of properties

    methods (Access = private)
    end % ... end of private methods
    
    methods (Access = public)
        function self = MedianFilter(m)
            % MEDIANFILTER is the constructor of MedianFilter, 
            % which creates the instances of the MedianFilter-class. 
            %
            % Syntax
            % obj = MedianFilter(m) 
            %
            % Inputs:
            % m    Number of sonar measurements
            self.m = m;
            self.data = zeros(1,m);
            self.xvec = zeros(1,m);
            for i = 1:m
                self.xvec(i) = i-1;
            end 
        end
        
        function z = filter(self,in)
            % FILTER is a function that returns the filtered altitude
            % measurement z. Also, a new sonar measurement in is added to
            % data, and the oldest sonar measurement is removed. 
            %
            % Syntax
            % obj = MedianFilter(m)
            % z = filter(self,in)
            %
            % Inputs
            % in   new sonar measurement
            if ~isscalar(in)
                z = in;
                return
            end
            filterData = sort(self.data, self.m);
            z = filterData(ceil(self.m/2)); % The median filtered heigh
            self.data(self.m) = z;
            self.data = [self.data(2:end),in]; % Add new measurement, remove oldest measurement
        end
                
        function out = addOutlier(~,in,probability)
            % ADDOUTLIER is a function that adds an outlier to a given 
            % input in, which represent the measurements of a sensor 
            % mounted on the quadcopter. This outlier is added with a 
            % probability given by input parameter probability (in 
            % percent).
            %
            % Syntax
            % obj = MedianFilter(m)  
            % out = obj.addOutlier(in,probability)
            %
            % Input arguments:
            % in    Sensor 
            if ~isscalar(in)
                out = in;
                return
            end
            s = rand;
            if s > 1-probability/100/2
                q = 1;
            elseif s < probability/100/2
                q = -1; 
            else
                q = 0; 
            end
            outlier = randi([1,5]); % The outlier is a value between 1 and 5 meter
            out = in + outlier*q; % Sonar measurement + outlier (noise)
        end
            
        function m = getm(self)
            % GETM is a getter method that returns the matrix number of 
            % stored sonar measurements m.
            % m is stored internally, therefore no inputs are required.
            %
            % Syntax
            % obj = MedianFilter(m)  
            % m = obj.getm()
            %
            % Input arguments:
            % None
            m = self.m;
        end
    end % ... end of public properties
end % ... end of class definition




