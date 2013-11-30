classdef LKF < kalman_filter_interface
    properties
        lnr_sys
    end
    methods
        function obj = LKF(lnr_sys)
            obj.lnr_sys = lnr_sys;
        end
        function b_next = estimate(b,U,Zg,lnr_sys_for_prd,lnr_sys_for_update)
            if nargin < 5
                error('Ali: The linearized systems has to be provided for LKF.')
            end
            b_prd = LKF.predict(b,U,lnr_sys_for_prd);
            b_next = LKF.update(b_prd,Zg,lnr_sys_for_update);
        end
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