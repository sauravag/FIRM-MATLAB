classdef Finite_time_LQR_class < LQR_interface
    %LQR_class encapsulates the LQR controller.
    
    properties
        kf; % period
    end
    
    
    methods
        function obj = Finite_time_LQR_class(lnr_sys_inp, lnr_pts_inp)
            obj.kf = size(lnr_sys_inp, 2);  % period
            obj.lnr_pts = lnr_pts_inp;
            obj.Feedback_gains = obj.generate_Feedback_gains(lnr_sys_inp);
        end
        
        function [u , reliable] = generate_feedback_control(obj, b, k)
            
            xp = obj.lnr_pts(k).x; % planned x (or target point) or linearization point.
            
            est_OF_error = b.est_mean.compute_distance_for_control(xp);
            
            reliable = obj.is_in_valid_linearization_region(est_OF_error);
            
            feedback_gain = obj.Feedback_gains{k};
            
            dU = - feedback_gain*est_OF_error;
            
            up = obj.lnr_pts(k).u; % planned up, which usually (or maybe always) must be zero in stationary LQG setting.
            
            u = up + dU;
        end
    end
    
    methods (Access = private)
        function feedbak_gains = generate_Feedback_gains(obj, lnr_sys_seq)
            % memory preallocation
            feedbak_gains = cell(1,obj.kf);
            % Solving Backward Riccati to compute Time-varying gains
            W_xf = LQR_interface.Final_state_cost;
            W_x = LQR_interface.state_cost;
            W_u = LQR_interface.control_cost;
            S = W_xf;
            
            for k = obj.kf : -1 : 1 % we must solve this Riccati BACKWARDS
                
                A = lnr_sys_seq(k).A; % A at the planned point at time k
                B = lnr_sys_seq(k).B; % B at the planned point at time k
                
                feedbak_gains{k} = (B'*S*B + W_u) \ B'*S*A; %LF is the feedback gain
                S = W_x + A'*S*A - A'*S*B*feedbak_gains{k};
                
                % if you want to save the S matrix too, you can use
                % following code instead of one in above.
                % LF{k} = + inv(B'*S(:,:,k+1)*B + W_u)*B'*S(:,:,k+1)*A; %LF is the feedback gain
                % S(:,:,k) = W_x + A'*S(:,:,k+1)*A - A'*S(:,:,k+1)*B*LF{k};
            end
        end
        function YesNo = is_in_valid_linearization_region(obj,est_OF_error)
            YesNo = all(abs(est_OF_error) < obj.valid_lnr_domain); % never forget the "absolute value operator" in computing distances.
        end
    end
    
end
