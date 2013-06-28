classdef LQR_class < handle
    %LQR_class encapsulates the LQR controller.
    
    % Note that because the class is defined as a handle class, the
    % properties must be defined such that they are do not change from an 
    % object to another one.
    properties (Constant = true)
        state_cost = user_data_class.par.state_cost;
        control_cost = user_data_class.par.control_cost;
        Final_state_cost = user_data_class.par.Final_state_cost;
        valid_lnr_domain = user_data_class.par.valid_linearization_domain;
    end
    methods (Static = true)
        function Lss = stationary_gain(lnr_sys)
            W_x=LQR_class.state_cost * user_data_class.par.state_cost_ratio_for_stationary_case; % note that "state_cost" is for the trajectory tracking. Usually in point stabilization, we need more force on state and less on control. So, we multiply the "state_cost" to an appropriate ratio. Note that this ratio has to be greater than 1.
            W_u=LQR_class.control_cost * user_data_class.par.control_cost_ratio_for_stationary_case; % note that "control_cost" is for the trajectory tracking. Usually in point stabilization, we need more force on state and less on control. So, we multiply the "control_cost" to an appropriate ratio. Note that this ratio has to be LESS than 1.
            
            Ass = lnr_sys.A;
            Bss = lnr_sys.B;
            
            [Sss,eig_ss,Lss,report] = dare(Ass,Bss,W_x,W_u);
            error_checking_for_DARE(Sss,eig_ss,Lss,report)
        end
        function LF_seq = time_varying_gains(lnr_sys_seq,kf)
            % I think the "kf" in the input arguments is not needed as it
            % is the length of the "lnr_sys_seq". Also, it make the
            % function similar to the "periodic_gains".
            
            % memory preallocation
            LF_seq = cell(1,kf);
            % Solving Backward Riccati to compute Time-varying gains
            W_xf = LQR_class.Final_state_cost;
            W_x = LQR_class.state_cost;
            W_u = LQR_class.control_cost;
            S = W_xf;
            
            for k = kf : -1 : 1 % we must solve this Riccati BACKWARDS
                
                A = lnr_sys_seq(k).A; % A at the planned point at time k
                B = lnr_sys_seq(k).B; % B at the planned point at time k
                
                LF_seq{k} = (B'*S*B + W_u) \ B'*S*A; %LF is the feedback gain
                S = W_x + A'*S*A - A'*S*B*LF_seq{k};
                
                % if you want to save the S matrix too, you can use
                % following code instead of one in above.
                % LF{k} = + inv(B'*S(:,:,k+1)*B + W_u)*B'*S(:,:,k+1)*A; %LF is the feedback gain
                % S(:,:,k) = W_x + A'*S(:,:,k+1)*A - A'*S(:,:,k+1)*B*LF{k};
            end
        end
        function LF_period = periodic_gains(lnr_sys_periodic)
            T = size(lnr_sys_periodic , 2);  % period
            LSS = lnr_sys_periodic;
            W_x=LQR_class.state_cost;
            W_u=LQR_class.control_cost;
            
            A_DPRE_LQR = nan(  size(LSS(1).A,1)  ,  size(LSS(1).A,2)  ,  T  );
            B_DPRE_LQR = nan(  size(LSS(1).B,1)  ,  size(LSS(1).B,2)  ,  T  );
            Q_DPRE_LQR = repmat(W_x  ,  [1,1,T]);
            R_DPRE_LQR = repmat(W_u ,  [1,1,T]);
            for k=1:T  % In this for loop, the system matrices names are changed to the ones that are needed in DPRE solver for LQRing.
                A_DPRE_LQR(:,:,k) = LSS(k).A;
                B_DPRE_LQR(:,:,k) = LSS(k).B;
            end
            [X_DPRE_LQR,K_DPRE_LQR] = dpre(A_DPRE_LQR,B_DPRE_LQR,Q_DPRE_LQR,R_DPRE_LQR);   %#ok<ASGLU>
            %  S_LQR =  X_DPRE_LQR;
            LF_period = K_DPRE_LQR;  % LQR periodic Feedback gain
        end
    end
end
