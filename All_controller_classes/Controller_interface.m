classdef Controller_interface
    %   LQG_stationary_class encapsulates the Stationary Linear Quadratic Gaussian Controller
    properties (Abstract)
        Estimator
        Separated_controller
    end
    
    methods (Abstract)
        [nextBelief, reliable] = executeOneStep(obj,oldBelief,noise_mode)
        nextHyperBelief = propagateHyperBelief(obj,oldHyperBelief)
    end
    
end