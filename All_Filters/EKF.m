classdef EKF < kalman_filter_interface
    methods
        function b_next = estimate(b,U,Zg)
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
            b_prd = EKF.predict(b,U,lnr_sys_for_prediction);
            
            lnr_pts_for_update.x = b_prd.est_mean.val; % in EKF, in update stage, we linearize about prediction mean
            lnr_pts_for_update.v = zeros(ObservationModel_class.obsNoiseDim,1);
            lnr_sys_for_update = Linear_system_class(lnr_pts_for_update);
            
            b_next = EKF.update(b_prd,Zg,lnr_sys_for_update);
        end
    end
end