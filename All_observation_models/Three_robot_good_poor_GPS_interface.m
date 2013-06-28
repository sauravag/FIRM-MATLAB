classdef Three_robot_good_poor_GPS_interface < ObservationModel_interface
    
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        sigma = 0.5; % The std of good GPS
        alpha = 30; % The ratio of std of good and bad GPS
    end
    properties (Constant, Abstract) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        obsDim;
        obsNoiseDim; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
    end
    properties
        plot_handle
    end
    
    methods (Static, Abstract)
        z = h_func(x,v)
        H = dh_dx_func(x,v)
        M = dh_dv_func(x,v)
        V = generate_observation_noise(x)
        R_team = noise_covariance(x_team)
        innov = compute_innovation(Xprd,Zg)
    end
    
    methods (Static = true)
        function tmp_prop = costant_property_constructor()
            tmp_prop.tmp_plot_handle = [];
        end
        function obj = draw(obj)
            obj.plot_handle = [];
        end
        function obj = delete_plot(obj)
            delete(obj.plot_handle)
            obj.plot_handle = [];
        end
    end
    
end