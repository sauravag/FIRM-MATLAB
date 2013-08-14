classdef SKF < kalman_filter_interface
    properties
        stationaryGain
        stationaryCov
    end
    methods
        function obj = SKF(lnr_sys)
            [obj.stationaryGain,~,obj.stationaryCov] = SKF.stationary_gain_and_covariances(lnr_sys);
        end
        function b_next = estimate(obj,b,U,Zg,lnr_sys_for_prd,lnr_sys_for_update)
            if nargin < 5
                error('Ali: The linearized systems has to be provided for LKF.')
            end
            b_prd = obj.predict(b,U,lnr_sys_for_prd);
            b_next = obj.update(b_prd,Zg,lnr_sys_for_update);
        end
        function b_next = StationaryKF_estimate(obj,b,U,Zg,lnr_sys,Stationary_Kalman_gain)
            disp('There is no mathmatical basis for the stationary KF if your system is not linear. So, do not use this. Use LKF and provide the stationary linear system as its inputs both for prediction and update. The problem with stationary KF is that the estimation covariance can become unsymmetric easily.')
            if nargin ~= 5
                error('Ali: In StationaryKF the linearized system and the stationary Kalman gain have to be provided as the function inputs.')
            end
            b_prd = obj.prediction(b,U,lnr_sys);
            % The update function in StationaryKF is different from the
            % update function in LKF and EKF.
            b_next = obj.update_with_stationary_gain(b_prd,Zg,lnr_sys,Stationary_Kalman_gain);
        end
        function b = update_with_stationary_gain(obj,b_prd,Zg,lnr_sys,stationary_Kalman_gain)
            % lnr_sys is the linear or linearized system, Kalman filter is
            % designed for.
            H = lnr_sys.H;
            % R = lnr_sys.R; % since we use the stationary Kalman gain as
            % the input here, we do not need the matrix R.
            Pprd = b_prd.est_cov;
            % I think in following line changing "inv" to "pinv" fixes possible
            % numerical issues
            KG = stationary_Kalman_gain; %KG is the "Kalman Gain"
            
            Xprd = b_prd.est_mean.val;
            innov = ObservationModel_class.compute_innovation(Xprd,Zg);
            Xest_next = Xprd+KG*innov;
            Pest_next = Pprd-KG*H*Pprd;
            
            % Due to the usage of approximate Kalman gain in stationary
            % kalman filter (Actually, we do not compute the gain based on
            % the current covariance. and we compute it based on the
            % stationary covariance), thus the estimation covariance can
            % get too unsymmetic.
            if max(max(abs(Pest_next-Pest_next')))<1e-12
                Pest_next = (Pest_next+Pest_next')/2;  %eliminating small unsymmetricity
            else
                % note that you should never remove following error. If you
                % need a symmetric covariance, you must use the "update"
                % function that computes the Kalman gain based on the
                % predicted covariance.
                error('AliFW: Robot is out of the valid linearization domain of the "stationary LQG controller". As a result, estimation covariance is too unsymmetric!!')
            end
            
            b = belief(state(Xest_next),Pest_next);
        end
    end
    methods(Static)
         function [Kss_correct,Pprd_ss,Pest_ss] = stationary_gain_and_covariances(lnr_sys)
            Ass = lnr_sys.A;
            %Bss is not needed
            Gss = lnr_sys.G;
            Qss = lnr_sys.Q;
            Hss = lnr_sys.H;
            Rss = lnr_sys.R;
            
            [Pprd_ss,eig_ss,Kss_wrong,report] = dare(Ass',Hss',Gss*Qss*Gss',Rss);  %#ok<ASGLU,NASGU> % Look at the footnote in page 194 at Dan Simon's book to see why this K matrix is wrong.
            Kss_correct=(Pprd_ss*Hss')/(Hss*Pprd_ss*Hss'+Rss);
            Pest_ss=Pprd_ss-(Pprd_ss*Hss')*inv(Hss*Pprd_ss*Hss'+Rss)*Hss*Pprd_ss;  %#ok<MINV>, Here, we use Joseph form to ensure the symmetricity of the covariance matrix.
            % In the following we check if we converge to the same covariance or not. It is just a test
            fast_running = 1; % if this is 1, following block is not executed.
            if fast_running == 0
                Pprd_test=eye(size(Pprd_ss,1));
                for i_test=1:3000
                    Pest_test=Pprd_test-Pprd_test*Hss'*inv(Hss*Pprd_test*Hss'+Rss)*Hss*Pprd_test; %#ok<MINV>
                    Pprd_test=Ass*Pest_test*Ass'+Gss*Qss*Gss';
                end
                
                if abs(max(max(Pprd_ss-Pprd_test)))>1.0e-6 || abs(max(max(Pest_ss-Pest_test)))>1.0e-6;
                    error('There are some issues with convergence')
                end
            end
        end
    end
end