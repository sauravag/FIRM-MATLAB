classdef Periodic_LQR_class < LQR_interface
    %Periodic_LQR_class encapsulates the LQR controller.
    
    properties
        T; % period
    end

    methods
        function obj = Periodic_LQR_class(lnr_sys_periodic, lnr_pts_inp)
            obj.T = size(lnr_sys_periodic , 2);  % period
            obj.lnr_pts = lnr_pts_inp;
            obj.Feedback_gains = obj.generate_Feedback_gains(lnr_sys_periodic);
        end
        function [u , reliable] = generate_feedback_control(obj, b, k)
            k = mod(k,obj.T); % This line is a crucial line, and makes the time periodic by the period "T".
            if k==0, k = obj.T; end
            xp = obj.lnr_pts(k).x; % planned x (or target point) or linearization point.
            
            est_OF_error = b.est_mean.compute_distance_for_control(xp);
            disp('Warning--> Periodic LQR Class: Only Aircraft Kinematic model has compute_distance_for_control implemented')
            
            reliable = obj.is_in_valid_linearization_region(est_OF_error);
            
            feedback_gain = obj.Feedback_gains(:,:,k);
            
            dU = - feedback_gain*est_OF_error;
            
            up = obj.lnr_pts(k).u; % planned up, which usually (or maybe always) must be zero in stationary LQG setting.
            
            u = up + dU;
        end
    end
    
    
    methods (Access = private)
        function LF_period = generate_Feedback_gains(obj, lnr_sys_periodic)
            LSS = lnr_sys_periodic;
            W_x=LQR_interface.state_cost;
            W_u=LQR_interface.control_cost;
            
            A_DPRE_LQR = nan(  size(LSS(1).A,1)  ,  size(LSS(1).A,2)  ,  obj.T  );
            B_DPRE_LQR = nan(  size(LSS(1).B,1)  ,  size(LSS(1).B,2)  ,  obj.T  );
            Q_DPRE_LQR = repmat(W_x  ,  [1,1,obj.T]);
            R_DPRE_LQR = repmat(W_u ,  [1,1,obj.T]);
            for k=1:obj.T  % In this for loop, the system matrices names are changed to the ones that are needed in DPRE solver for LQRing.
                A_DPRE_LQR(:,:,k) = LSS(k).A;
                B_DPRE_LQR(:,:,k) = LSS(k).B;
            end
            [X_DPRE_LQR,K_DPRE_LQR] = dpre(A_DPRE_LQR,B_DPRE_LQR,Q_DPRE_LQR,R_DPRE_LQR);   %#ok<ASGLU>
            %  S_LQR =  X_DPRE_LQR;
            LF_period = K_DPRE_LQR;  % LQR periodic Feedback gain
        end
        function YesNo = is_in_valid_linearization_region(obj,est_OF_error)
            YesNo = all(abs(est_OF_error) < obj.valid_lnr_domain); % never forget the "absolute value operator" in computing distances.
        end
    end
end
