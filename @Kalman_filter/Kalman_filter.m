classdef Kalman_filter
    %Kalman_filter
    %   This class implements both EKF and LKF
    
    methods (Static = true)
        function b_prd = prediction(b,U,lnr_sys)
            % lnr_sys is the linear or linearized system, Kalman filter is
            % designed for.
            
            A = lnr_sys.A;
            %B = lnr_sys.B; % not needed in this function
            G = lnr_sys.G;
            Q = lnr_sys.Q;
            
            % Pprd=(A-B*L)*Pest_old*(A-B*L)'+Q;  % wroooooong one
            
            Xest_old = b.est_mean.val;
            Pest_old = b.est_cov;
            
            zerow = zeros(MotionModel_class.wDim,1);
            Xprd = MotionModel_class.f_discrete(Xest_old,U,zerow);
            
            % I removed following display, because it comes up there too
            % much times!
            %disp('AliFW: for LKF, it seems more correct to use linear prediction step.')
            
            
            %Xprd = A*Xest_old+B*U; % This line is veryyyyyyyyyy
            %wroooooooooooooooooooooooooong. Because, this equation only
            %holds for state error NOT the state itself.
            Pprd = A*Pest_old*A'+G*Q*G';
            
            b_prd = belief(state(Xprd),Pprd);
        end
        function b = update(b_prd,Zg,lnr_sys)
            % lnr_sys is the linear or linearized system, Kalman filter is
            % designed for.
            H = lnr_sys.H;
            R = lnr_sys.R;
            Pprd = b_prd.est_cov;
            % I think in following line changing "inv" to "pinv" fixes possible
            % numerical issues
            KG = (Pprd*H')/(H*Pprd*H'+R); %KG is the "Kalman Gain"
            
            Xprd = b_prd.est_mean.val;
            innov = ObservationModel_class.compute_innovation(Xprd,Zg);
            Xest_next = Xprd+KG*innov;
            Pest_next = Pprd-KG*H*Pprd;
            b = belief(state(Xest_next),Pest_next);
            bout = b.apply_differentiable_constraints(); % e.g., quaternion norm has to be one
            b=bout;
        end
        function b = update_with_periodic_gain(b_prd,Zg,lnr_sys,periodic_Kalman_gain)
            % lnr_sys is the linear or linearized system, Kalman filter is
            % designed for.
            H = lnr_sys.H;
            % R = lnr_sys.R; % since we use the periodic Kalman gain as
            % the input here, we do not need the matrix R.
            Pprd = b_prd.est_cov;
            % I think in following line changing "inv" to "pinv" fixes possible
            % numerical issues
            KG = periodic_Kalman_gain; %KG is the "Kalman Gain"
            
            Xprd = b_prd.est_mean.val;
            innov = ObservationModel_class.compute_innovation(Xprd,Zg);
            Xest_next = Xprd+KG*innov;
            Pest_next = Pprd-KG*H*Pprd;
            
            % Due to the usage of approximate Kalman gain in periodic
            % kalman filter (Actually, we do not compute the gain based on
            % the current covariance. and we compute it based on the
            % periodic pre-computed covariance), thus the estimation covariance can
            % get too unsymmetic.
            if max(max(abs(Pest_next-Pest_next')))<1e-12
                Pest_next = (Pest_next+Pest_next')/2;  %eliminating small unsymmetricity
            else
                % note that you should never remove following error. If you
                % need a symmetric covariance, you must use the "update"
                % function that computes the Kalman gain based on the
                % predicted covariance.
                error('AliFW: Robot is out of the valid linearization domain of the "periodic LQG controller". As a result, estimation covariance is too unsymmetric!!')
            end
            
            b = belief(state(Xest_next),Pest_next);
        end
        function b = update_with_stationary_gain(b_prd,Zg,lnr_sys,stationary_Kalman_gain)
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
        function b_next = EKF_estimate(b,U,Zg)
            if nargin > 3
                error('Ali: EKF does not need linearized system. Because EKF computes its own linearized system.')
            end
            % EKF computes its own linearized system. EKF linearizes
            % the system twice: once in prediction phase, and again in
            % update phase.
            lnr_pts_for_prediction.x = b.est_mean.val; % in EKF, in prediction stage, we linearize about estimation mean
            lnr_pts_for_prediction.u = U; % in EKF, we linearize about the up+dU
            lnr_pts_for_prediction.w = zeros(MotionModel_class.wDim,1);
            lnr_sys_for_prediction = Linear_system_class(lnr_pts_for_prediction);
            b_prd = Kalman_filter.prediction(b,U,lnr_sys_for_prediction);
            
            lnr_pts_for_update.x = b_prd.est_mean.val; % in EKF, in update stage, we linearize about prediction mean
            lnr_pts_for_update.v = zeros(ObservationModel_class.obsNoiseDim,1);
            lnr_sys_for_update = Linear_system_class(lnr_pts_for_update);
            
            b_next = Kalman_filter.update(b_prd,Zg,lnr_sys_for_update);
        end
        function b_next = LKF_estimate(b,U,Zg,lnr_sys_for_prd,lnr_sys_for_update)
            if nargin < 5
                error('Ali: The linearized systems has to be provided for LKF.')
            end
            b_prd = Kalman_filter.prediction(b,U,lnr_sys_for_prd);
            b_next = Kalman_filter.update(b_prd,Zg,lnr_sys_for_update);
        end
        function b_next = StationaryKF_estimate(b,U,Zg,lnr_sys,Stationary_Kalman_gain)
            disp('There is no mathmatical basis for the stationary KF if your system is not linear. So, do not use this. Use LKF and provide the stationary linear system as its inputs both for prediction and update. The problem with stationary KF is that the estimation covariance can become unsymmetric easily.')
            if nargin ~= 5
                error('Ali: In StationaryKF the linearized system and the stationary Kalman gain have to be provided as the function inputs.')
            end
            b_prd = Kalman_filter.prediction(b,U,lnr_sys);
            % The update function in StationaryKF is different from the
            % update function in LKF and EKF.
            b_next = Kalman_filter.update_with_stationary_gain(b_prd,Zg,lnr_sys,Stationary_Kalman_gain);
        end
        function b_next = PeriodicKF_estimate(b,U,Zg,lnr_sys_for_prd,lnr_sys_for_update,Periodic_Kalman_gain)
            disp('There is no mathmatical basis for the periodic KF if your system is not linear. So, do not use this. Use LKF and provide the periodic linear system as its inputs. The problem with periodic KF is that the estimation covariance can become unsymmetric easily.')
            if nargin ~= 6
                error('Ali: In Periodic_KF the linearized system for prediction and update and the Kalman Gain have to be provided as the function inputs.')
            end
            b_prd = Kalman_filter.prediction(b,U,lnr_sys_for_prd);
            % The update function in StationaryKF is different from the
            % update function in LKF and EKF.
            b_next = Kalman_filter.update_with_periodic_gain(b_prd,Zg,lnr_sys_for_update,Periodic_Kalman_gain);
        end
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
        function [K_periodic_correct , Pprd_periodic, Pest_periodic] = periodic_gain_and_covariances(lnr_sys_periodic)
            % The "lnr_sys_periodic" is a "1 by T" array of "linear system"
            % objects.
            T = size(lnr_sys_periodic , 2);  % period
            LSS = lnr_sys_periodic;
            % Memory preallocation. % To see the following relation about
            % matrix sizes, you have to look at the "dpre" function
            % documentation.
            A_DPRE_KF = nan(  size(LSS(1).A',1)  ,  size(LSS(1).A',2)  ,  T  );
            B_DPRE_KF = nan(  size(LSS(1).H',1)  ,  size(LSS(1).H',2)  ,  T  );
            Q_DPRE_KF = nan(  size(LSS(1).G*LSS(1).Q*LSS(1).G',1)  ,  size(LSS(1).G*LSS(1).Q*LSS(1).G',2)  ,  T  );
            R_DPRE_KF = nan(  size(LSS(1).M*LSS(1).R*LSS(1).M',1)  ,  size(LSS(1).M*LSS(1).R*LSS(1).M',2)  ,  T  );
            
            for k=1 : T % In this for loop, the system matrices names are changed to the ones that are needed in DPRE solver for Kalman Filtering.
                % time reversing % This is an exteremly tricky part. To do it
                % correctly you must have to write a simple example on the paper
                % with for example T = 5 to see how this time reversing works. You
                % can see the math in latex in dpre function.
                if k==T
                    new_k = T;
                else
                    new_k = T - k;
                end
                % following follows from $$\overline{k}=T-k $$ and $$ E_k = I,~S_k = 0,~X_k = P^-_{\overline{k}+1},~Q_k = G_{\overline{k}}Q_{\overline{k}}G^T_{\overline{k}},~ R_k = M_{\overline{k}}R_{\overline{k}}M_{\overline{k}}^T,~A_k=A^T_{\overline{k}},~B_k=H^T_{\overline{k}} $$
                A_DPRE_KF(:,:,k) = LSS(new_k).A';
                B_DPRE_KF(:,:,k) = LSS(new_k).H';
                Q_DPRE_KF(:,:,k) = LSS(new_k).G*LSS(new_k).Q*LSS(new_k).G';
                R_DPRE_KF(:,:,k) = LSS(new_k).M*LSS(new_k).R*LSS(new_k).M';
            end
            
            [X_DPRE_KF,K_DPRE_KF_wrong] = dpre(A_DPRE_KF,B_DPRE_KF,Q_DPRE_KF,R_DPRE_KF,[],[],1e-6,1e3);  %#ok<NASGU>
            Pprd_periodic = flipdim(X_DPRE_KF,3);  % reverse the time  k = T - k_new
            
            Pest_periodic = nan(size(LSS(1).A,1)  , size(LSS(1).A,1)  , T);
            K_periodic_correct = nan(size( LSS(1).A,1)  , size(LSS(1).H,1) , T );
            for k = 1:T
                %  K_periodic_wrong(:,:,k) = A_DPRE_KF(:,:,k)\(K_DPRE_KF(:,:,k)');  % inv(A_DPRE_KF(:,:,k)) * K_DPRE_KF(:,:,k)'; % Periodic Kalman Gain
                K_periodic_correct(:,:,k) = (Pprd_periodic(:,:,k)*LSS(k).H')/(LSS(k).H*Pprd_periodic(:,:,k)*LSS(k).H'+LSS(k).M*LSS(k).R*LSS(k).M');
                Pest_periodic(:,:,k) = (eye(MotionModel_class.stDim)-K_periodic_correct(:,:,k)*LSS(k).H)*Pprd_periodic(:,:,k);  % Periodic estimation covariance
            end
        end
        function KGain_seq_for_LKF = Kalman_Gain_seq_for_LKF(lnr_sys_seq,initial_Pest,kf) %#ok<INUSD,STOUT>
            error('So far, there is no need for this function to be used.')
            % memory preallocation
            KG = cell(1,kf+1); %#ok<UNRCH>
            % Solving Forward Riccati to compute Time-varying gains
            Pest = initial_Pest;
            
            for k = 1 : kf % we must solve this Riccati BACKWARDS
                % Jacobians at time k
                A = lnr_sys_seq(k).A;
                G = lnr_sys_seq(k).G;
                Q = lnr_sys_seq(k).Q;
                % Jacobians at time k+1
                H = lnr_sys_seq(k+1).H;
                R = lnr_sys_seq(k+1).R;
                
                Pprd = A*Pest*A'+G*Q*G';
                KG{k+1} = (Pprd*H')/(H*Pprd*H'+R); %KG is the "Kalman Gain" which is associated with the time k+1.
                Pest = Pprd-KG{k+1}*H*Pprd;
            end
            KGain_seq_for_LKF = KG;
            
        end
    end
    
end