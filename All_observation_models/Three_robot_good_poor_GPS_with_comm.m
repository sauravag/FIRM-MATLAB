classdef Three_robot_good_poor_GPS_with_comm < Three_robot_good_poor_GPS_interface
    
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        tmp_prop = Three_robot_good_poor_GPS_with_comm.costant_property_constructor();  % This property is not needed!! I only added it because I could not find any other way to initialize the rest of constant properties.
        obsDim = 10;
        obsNoiseDim = 10; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
        eta_com = 0.11*5;
        sigma_b_com = 0.1*.015;
    end
    
    methods (Static = true)
        function z_team = h_func(x_team,v_team)
            d21 = x_team(3:4) - x_team(1:2);
            d31 = x_team(5:6) - x_team(1:2);
            d32 = x_team(5:6) - x_team(3:4);
            z_team(1:6, 1) = x_team(1:6)+v_team(1:6);
            z_team(7:10, 1) = [d21;d31]; % note that "z_team" must be a column vector.
%             norm(d21)+v_team(7); % note that "z_team" must be a column vector.
%             z_team(8, 1) = norm(d31)+v_team(8); % note that "z_team" must be a column vector.
%             z_team(9, 1) = norm(d32)+v_team(9); % note that "z_team" must be a column vector.
%             z_team(10, 1) = d21(1)+v_team(10); % note that "z_team" must be a column vector.
%             z_team(11, 1) = d31(2)+v_team(11); % note that "z_team" must be a column vector.
        end
        function H_team = dh_dx_func(x_team,v_team) %#ok<INUSD>
            d21 = x_team(3:4) - x_team(1:2);
            d31 = x_team(5:6) - x_team(1:2);
            d32 = x_team(5:6) - x_team(3:4);
%             tmp1 = (d21'/norm(d21))*[-1 0 1 0 0 0 ; 0 -1 0 1 0 0];
%             tmp2 = (d31'/norm(d31))*[-1 0 0 0 1 0 ; 0 -1 0 0 0 1];
%             tmp3 = (d32'/norm(d32))*[0 0 -1 0 1 0 ; 0 0 0 -1 0 1];
            tmp = [-1 0 1 0 0 0; 0 -1 0 1 0 0 ; -1 0 0 0 1 0 ; 0 -1 0 0 0 1];
%             tmp5 = [0 -1 0 0 0 1];
            H_team = [eye(6) ; tmp];
        end
        function M = dh_dv_func(x,v) %#ok<INUSD>
            M = eye(10);
        end
        function R_team = noise_covariance(x_team)
            d21 = x_team(3:4) - x_team(1:2);
            d31 = x_team(5:6) - x_team(1:2);
            d32 = x_team(5:6) - x_team(3:4);
            
            sigma_good_GPS = Three_robot_good_poor_GPS_with_comm.sigma;
            alpha = Three_robot_good_poor_GPS_with_comm.alpha;
            sigma_bad_GPS = alpha * sigma_good_GPS;
            
            eta_comm = Three_robot_good_poor_GPS_with_comm.eta_com;
            sigma_b_comm = Three_robot_good_poor_GPS_with_comm.sigma_b_com;
            
            sigma_com21 = eta_comm * norm(d21) + sigma_b_comm;
            sigma_com31 = eta_comm * norm(d31) + sigma_b_comm;
            sigma_com32 = eta_comm * norm(d32) + sigma_b_comm;
            
            std_vector = [sigma_good_GPS ; 
                                      sigma_good_GPS ;
                                      sigma_bad_GPS ;
                                      sigma_bad_GPS ;
                                      sigma_bad_GPS ;
                                      sigma_bad_GPS ;
                                      sigma_com21 ;
                                      sigma_com21 ;
                                      sigma_com31 ;
                                      sigma_com31 ];
            
            R_team=diag(std_vector.^2);
        end
        function V = generate_observation_noise(x_team)
            noise=randn(10,1);
            V = noise.*diag((Three_robot_good_poor_GPS_with_comm.noise_covariance(x_team)).^(1/2));
        end
        function innov = compute_innovation(Xprd,Zg)
            V = zeros(10,1);
            Zprd = ObservationModel_class.h_func(Xprd,V);
            innov = Zg - Zprd;
        end
    end
    
end