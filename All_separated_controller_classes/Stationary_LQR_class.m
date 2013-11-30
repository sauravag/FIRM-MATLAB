classdef Stationary_LQR_class < LQR_interface
    %Stationary_LQR_class encapsulates the LQR controller.
    
    methods
        function obj = Stationary_LQR_class(lnr_sys_inp, lnr_pts_inp)
            obj.lnr_pts = lnr_pts_inp;
            obj.Feedback_gains = obj.generate_feedback_gain(lnr_sys_inp);
        end
        function [u , reliable] = generate_feedback_control(obj, b)
            xp = obj.lnr_pts.x; % planned x (or target point) or linearization point.
            est_OF_error = b.est_mean.signed_element_wise_dist(xp);  % this basically computes the "signed element-wise distance" between "b.est_mean" and "xp"
            reliable = obj.is_in_valid_linearization_region(est_OF_error);
            dU = - obj.Feedback_gains*est_OF_error;
            up = obj.lnr_pts.u; % planned up, which usually (or maybe always) must be zero in stationary LQG setting.
            u = up + dU;
        end
    end
    methods (Access = private)
        function feedbak_gain = generate_feedback_gain(obj, lnr_sys_inp)
            W_x=LQR_interface.state_cost * user_data_class.par.state_cost_ratio_for_stationary_case; % note that "state_cost" is for the trajectory tracking. Usually in point stabilization, we need more force on state and less on control. So, we multiply the "state_cost" to an appropriate ratio. Note that this ratio has to be greater than 1.
            W_u=LQR_interface.control_cost * user_data_class.par.control_cost_ratio_for_stationary_case; % note that "control_cost" is for the trajectory tracking. Usually in point stabilization, we need more force on state and less on control. So, we multiply the "control_cost" to an appropriate ratio. Note that this ratio has to be LESS than 1.
            
            Ass = lnr_sys_inp.A;
            Bss = lnr_sys_inp.B;
            
            [Sss,eig_ss,Lss_local,report] = dare(Ass,Bss,W_x,W_u);
            error_checking_for_DARE(Sss,eig_ss,Lss_local,report)
            feedbak_gain = Lss_local;
        end
        function YesNo = is_in_valid_linearization_region(obj,est_OF_error)
            YesNo = all(abs(est_OF_error) < obj.valid_lnr_domain); % never forget the "absolute value operator" in computing distances.
        end
    end
end
