classdef GPS_positional < ObservationModel_interface
    
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        R = diag([0.1 , 0.1 , 2 , 2 ,2 ,2 ,2, 2]); % observation noise covariance
    end
    properties (Constant)
        obsDim = state.dim;
        obsNoiseDim = state.dim; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
    end
    
    methods (Static = true)
        function handle_of_plot = draw()
            handle_of_plot = [];
        end
        function z = h_func(x,v)
            z = x+v;
        end
        function H = dh_dx_func(x,v) %#ok<INUSD>
            H = eye(GPS_positional.obsDim);
        end
        function M = dh_dv_func(x,v) %#ok<INUSD>
            M = eye(GPS_positional.obsNoiseDim);
        end
        function V = generate_observation_noise(x) %#ok<INUSD>
            noise=randn(GPS_positional.obsDim,1);
            V = noise.*diag((GPS_positional.R).^(1/2));
        end
        function R = noise_covariance(x) %#ok<INUSD>
            R = GPS_positional.R;
        end
        innov = compute_innovation(obj,Xprd,Zg)
    end
    
end