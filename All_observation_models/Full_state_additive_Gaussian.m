classdef Full_state_additive_Gaussian < ObservationModel_interface
    
    properties (Constant)
        tmp_prop = Full_state_additive_Gaussian.costant_property_constructor();  % I use this technique to initialize the costant properties in run-time. If I can find a better way to do it, I will update it, as it seems a little bit strange.
        obsDim = state.dim;
        obsNoiseDim = state.dim; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
        R = eye(state.dim)*0.0001; % observation noise covariance
        zeroNoise = zeros(state.dim,1);
    end
    
    methods (Static = true)
        function tmp_prop = costant_property_constructor()
            tmp_prop = [];
        end
        function handle_of_plot = draw()
            handle_of_plot = [];
        end
        function obj = delete_plot(obj)
        end
        function z = h_func(x,v)
            z = x+v;
        end
        function H = dh_dx_func(x,v) %#ok<INUSD>
            H = eye(Full_state_additive_Gaussian.obsDim);
        end
        function M = dh_dv_func(x,v) %#ok<INUSD>
            M = eye(Full_state_additive_Gaussian.obsNoiseDim);
        end
        function V = generate_observation_noise(x) %#ok<INUSD>
            noise=randn(Full_state_additive_Gaussian.obsDim,1);
            V = noise.*diag((Full_state_additive_Gaussian.R).^(1/2));
        end
        function R = noise_covariance(x) %#ok<INUSD>
            R = Full_state_additive_Gaussian.R;
        end
        function innov = compute_innovation(Xprd,Zg)
            innov = Zg - Xprd;
        end
    end
    
end