classdef Periodic_LQR_class
    %Periodic_LQR_class encapsulates the LQR controller.
    
    properties
        T; % period
        Periodic_Feedback_gain; % feedback gain
        lnr_pts_periodic;
    end
    properties (Constant = true)
        state_cost = user_data_class.par.state_cost;
        control_cost = user_data_class.par.control_cost;
        Final_state_cost = user_data_class.par.Final_state_cost;
        valid_lnr_domain = user_data_class.par.valid_linearization_domain;
    end
    methods
        function obj = Periodic_LQR_class(lnr_sys_periodic, lnr_pts_inp)
             obj.T = size(lnr_sys_periodic , 2);  % period
            obj.lnr_pts_periodic = lnr_pts_inp;
            obj.Periodic_Feedback_gain = obj.generate_periodic_feedback_gain(lnr_sys_periodic);
        end
        function [u , reliable] = generate_feedback_control(obj, b, k)
            k = mod(k,obj.T); % This line is a crucial line, and makes the time periodic by the period "T".
            if k==0, k = obj.T; end
            xp = obj.lnr_pts_periodic(k).x; % planned x (or target point) or linearization point.
            
            disp('Periodic LQR Class: Feedback Control only works for 7 dim system!!!!')
            %%%%%%%%%%%%%%
            % EST OF ERROR WITH QUAT PRODUCT %
             x11 = b.est_mean.val; % retrieve the value of the state vector
             x22 = xp; % retrieve the value of the state vector
             signed_dist_position = x11(1:3) - x22(1:3); % [X1-X2, Y1-Y2, Z1-Z2]'
             q_1 = x11(4:7)';
             q_2 = x22(4:7)';
             q21 = quatmultiply(q_1,quatinv(q_2)); % relative rotation quaternion from nominal to current
             signed_dist_vector = [signed_dist_position;q21'];

             est_OF_error = signed_dist_vector;
             est_OF_error(4)=0;
            %%%%%%%%%%%%%%
%             est_OF_error = b.est_mean.signed_element_wise_dist(xp);  % this basically computes the "signed element-wise distance" between "b.est_mean" and "xp"
            reliable = obj.is_in_valid_linearization_region(est_OF_error);
            feedback_gain = obj.Periodic_Feedback_gain(:,:,k);
            dU = - feedback_gain*est_OF_error;
            up = obj.lnr_pts_periodic(k).u; % planned up, which usually (or maybe always) must be zero in stationary LQG setting.
            u = up + dU;
        end
    end
    
    
    methods (Access = private)
        function LF_period = generate_periodic_feedback_gain(obj, lnr_sys_periodic)
            LSS = lnr_sys_periodic;
            W_x=Periodic_LQR_class.state_cost;
            W_u=Periodic_LQR_class.control_cost;
            
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
