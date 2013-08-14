classdef belief_interface
    % This class encapsulates the Gaussian belief concept.
    properties
        est_mean; %estimation mean
        est_cov; %estimation covariance
        ellipse_handle;
    end

    methods (Abstract)
        obj = draw(obj, varargin)
        obj = draw_CovOnNominal(obj, nominal_state, varargin)
    end
    
    methods
        function obj = belief_interface(arg1,arg2)
            % Class constructor that can take the belief object or a
            % (est_mean and est_cov) pair, as its input.
            if nargin==0 % initialize the belief with empty values
                obj.est_mean = state();
            end
            if nargin == 1
                obj = arg1;
            elseif nargin == 2
                obj.est_mean = arg1;
                obj.est_cov = arg2;
            end
        end
        function obj = delete_plot(obj,varargin)
            try % Avoid errors if the graphic object has already been deleted
                delete(obj.ellipse_handle);
            end
            obj.ellipse_handle = [];
            obj.est_mean = obj.est_mean.delete_plot(varargin{:});
        end
    end

end