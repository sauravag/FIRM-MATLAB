classdef ObservationModel_interface
    properties (Abstract, Constant) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        obsDim; % dimension of the observation vector
        obsNoiseDim; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
        % plot_handle; % handle of "drawings" associated with observation; This property cannot be Constant, so we removed it. Note that we do not want to observationModel class has a non-constant property, because there will a lot of copies of this class and we do not want it to occupy a lot of memory.
    end
    
    methods (Abstract) % These methods are all "static". But, we cannot write the "static", because if we do so, we cannot derive another "abstract" class from this class (due to Matlab's poor OOP). This issue is resolved easily by setting the "static" flag to "true" in the child classes.
        tmp_prop = costant_property_constructor() %#ok<NOIN>
        z = h_func(x,v)
        H = dh_dx_func(x,v)
        M = dh_dv_func(x,v)
        R = noise_covariance(x)
        V = generate_observation_noise(x)
        innov = compute_innovation(Xprd,Zg)
    end

    methods (Abstract) % Note that these methods will *NOT* be "static" in their child classes.
        obj = draw(obj);
        obj = delete_plot(obj);
    end
    
end