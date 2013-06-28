classdef ObservationModel_class < ObservationModel_class_two_robots_interface
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        num_robots  = user_data_class.par.state_parameters.num_robots;
        obsDim =7;
        obsNoiseDim = 7; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
        eta_com = 0.11*.15;
        sigma_b_com = 0.1*.15;
    end

    methods (Static = true)
        function z_team = h_func(x_team,v_team)
            d = x_team(4:5) - x_team(1:2);
            z_team(1:6, 1) = x_team(1:6)+v_team(1:6);
            z_team(7, 1) = norm(d)+v_team(7); % note that "z_team" must be a column vector.
        end
        function H_team = dh_dx_func(x_team,v_team) %#ok<INUSD>
            d = x_team(4:5) - x_team(1:2);
            tmp = (d'/norm(d))*[-1 0 0 1 0 0 ; 0 -1 0 0 1 0];
            H_team = [eye(ObservationModel_class.obsDim-1) ; tmp];
        end
        function M = dh_dv_func(x,v) %#ok<INUSD>
            % Jacobian of observation wrt observation noise.
            M = eye(ObservationModel_class.obsDim);
        end
        function R = noise_covariance(x_team)
            d = x_team(4:5) - x_team(1:2);
            sigma_x = ObservationModel_class.eta_x * abs(x_team(1) - ObservationModel_class.Lx) + ObservationModel_class.sigma_b_x;
            sigma_y = ObservationModel_class.eta_y * abs(x_team(5) - ObservationModel_class.Ly) + ObservationModel_class.sigma_b_y;
            sigma_com = ObservationModel_class.eta_com * norm(d) + ObservationModel_class.sigma_b_com;
            std_vector = [sigma_x ; ObservationModel_class.alpha_ratio * sigma_x ; ObservationModel_class.sigma_theta ; ObservationModel_class.alpha_ratio * sigma_y ; sigma_y ; ObservationModel_class.sigma_theta ; sigma_com];
            R=diag(std_vector.^2);
        end
        function V = generate_observation_noise(x)
            obsDim = ObservationModel_class.obsDim;%#ok<PROP>
            R = ObservationModel_class.noise_covariance(x);
            indep_part_of_obs_noise=randn(obsDim,1); %#ok<PROP>
            V = indep_part_of_obs_noise.*diag(R.^(1/2));
        end
    end
end