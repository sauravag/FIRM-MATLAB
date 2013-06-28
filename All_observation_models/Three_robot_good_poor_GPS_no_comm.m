classdef Three_robot_good_poor_GPS_no_comm < Three_robot_good_poor_GPS_interface
    
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        obsDim = 6;
        obsNoiseDim = 6; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
    end
    
    methods (Static = true)
        function z = h_func(x,v)
            z = x+v;
        end
        function H = dh_dx_func(x,v) %#ok<INUSD>
            H = eye(Three_robot_good_poor_GPS_no_comm.obsDim);
        end
        function M = dh_dv_func(x,v) %#ok<INUSD>
            M = eye(Three_robot_good_poor_GPS_no_comm.obsNoiseDim);
        end
        function V = generate_observation_noise(x)
            noise=randn(Three_robot_good_poor_GPS_no_comm.obsDim,1);
            V = noise.*diag((Three_robot_good_poor_GPS_no_comm.noise_covariance(x)).^(1/2));
        end
        function R_team = noise_covariance(x_team)   %#ok<INUSD>
            sigma_good_GPS = Three_robot_good_poor_GPS_no_comm.sigma;
            alpha_local = Three_robot_good_poor_GPS_no_comm.alpha;
            sigma_bad_GPS = alpha_local * sigma_good_GPS;
            
            std_vector = [sigma_good_GPS ; 
                                      sigma_good_GPS ;
                                      sigma_bad_GPS ;
                                      sigma_bad_GPS ;
                                      sigma_bad_GPS ;
                                      sigma_bad_GPS ];
            
            R_team=diag(std_vector.^2);
        end
        function innov = compute_innovation(Xprd,Zg)
            V = zeros(ObservationModel_class.obsNoiseDim,1);
            Zprd = ObservationModel_class.h_func(Xprd,V);
            innov = Zg - Zprd;
        end
    end
    
end