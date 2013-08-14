classdef LQG_interface < Controller_interface
    %   LQG_based_class, from which the "SLQG", "PLQG", and "FLQG" are derived.
    properties (Abstract)
        estimator
        separated_controller
        lnr_pts;
        lnr_sys;
    end
    
    methods (Abstract)
        [nextBelief, reliable] = executeOneStep(obj,oldBelief,noise_mode)
        nextHyperBelief = propagateHyperBelief(obj,oldHyperBelief)
    end

end