classdef Two_robots_no_communication < ObservationModel_class_two_robots_interface
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        num_robots  = user_data_class.par.state_parameters.num_robots;
        obsDim = 6;
        obsNoiseDim = 6; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
    end
    
    methods (Static = true)
        function z_team = h_func(x_team,v_team)
            z_team = x_team+v_team;
        end
        function H_team = dh_dx_func(x_team,v_team) %#ok<INUSD>
            H_team = eye(Two_robots_no_communication.obsDim);
        end
        function M = dh_dv_func(x,v) %#ok<INUSD>
            % Jacobian of observation wrt observation noise.
            M = eye(Two_robots_no_communication.obsDim);
        end
        function V = generate_observation_noise(x)
            obsDim = Two_robots_no_communication.obsDim;%#ok<PROP>
            R = Two_robots_no_communication.noise_covariance(x);
            indep_part_of_obs_noise=randn(obsDim,1); %#ok<PROP>
            V = indep_part_of_obs_noise.*diag(R.^(1/2));
        end
        function R = noise_covariance(x_team)
            sigma_x = Two_robots_no_communication.eta_x * abs(x_team(1) - Two_robots_no_communication.Lx) + Two_robots_no_communication.sigma_b_x;
            sigma_y = Two_robots_no_communication.eta_y * abs(x_team(5) - Two_robots_no_communication.Ly) + Two_robots_no_communication.sigma_b_y;
            std_vector = [sigma_x ; Two_robots_no_communication.alpha_ratio * sigma_x ; Two_robots_no_communication.sigma_theta ; Two_robots_no_communication.alpha_ratio * sigma_y ; sigma_y ; Two_robots_no_communication.sigma_theta ];
            R=diag(std_vector.^2);
        end
    end
end