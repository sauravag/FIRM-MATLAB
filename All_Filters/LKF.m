classdef LKF < kalman_filter_interface
    methods
        function b_next = estimate(b,U,Zg,lnr_sys_for_prd,lnr_sys_for_update)
            if nargin < 5
                error('Ali: The linearized systems has to be provided for LKF.')
            end
            b_prd = LKF.predict(b,U,lnr_sys_for_prd);
            b_next = LKF.update(b_prd,Zg,lnr_sys_for_update);
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