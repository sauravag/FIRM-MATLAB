classdef LQG_class
    % LQG encapsulates the time-varying Linear Quadratic Gaussian Controller
    % Note that this class can use a lot of memory depending on the length
    % of state trajectory. Thus, you should clear the objects of this class
    % when you do not need them anymore. You can do that using
    % "erase_from_memory" function.
    
    
    properties (Constant = true)
        valid_lnr_domain = user_data_class.par.valid_linearization_domain;
    end
    properties
        kf
        planned_lnr_pts_seq; % linearization points (planned trajectory and controls.)
        planned_lnr_sys_seq; % linear systems (planned systems.)
        L_seq; % feedback gain sequebce
        filter_mode; % this can be "EKF" or "LKF"
    end
    
    methods
        function obj = LQG_class(filter_mode_inp,nominal_trajectory)
            if nargin > 0
                obj.filter_mode = filter_mode_inp;
                obj.kf = size(nominal_trajectory.x , 2)-1;
                x_p = nominal_trajectory.x;
                u_p = nominal_trajectory.u;
                
                if size(u_p,2)<obj.kf+1, u_p = [u_p,nan(MotionModel_class.ctDim,1)]; end
                
%                 [u_p,obj.kf] = MotionModel_class.compute_planned_control(x_initial,x_final);
%                 x_p = MotionModel_class.compute_planned_traj(x_initial,u_p,obj.kf);
                w_p = [zeros(MotionModel_class.wDim,obj.kf),nan(MotionModel_class.wDim,1)];
                v_p = [zeros(ObservationModel_class.obsNoiseDim,obj.kf),nan(ObservationModel_class.obsNoiseDim,1)];
                
                % memory preallocation
                obj.planned_lnr_sys_seq = Linear_system_class.empty;
                
                for k = 1 : obj.kf+1
                    obj.planned_lnr_pts_seq(k).x = x_p(:,k);
                    obj.planned_lnr_pts_seq(k).u = u_p(:,k);
                    obj.planned_lnr_pts_seq(k).w = w_p(:,k);
                    obj.planned_lnr_pts_seq(k).v = v_p(:,k);
                    obj.planned_lnr_sys_seq(k) = Linear_system_class(obj.planned_lnr_pts_seq(k));
                end
                obj.L_seq = LQR_class.time_varying_gains(obj.planned_lnr_sys_seq,obj.kf);
            end
        end
%         function obj = LQG_class(filter_mode_inp,x_initial,x_final)
%             % AliFW: This function has to be changes as soon as possible.
%             % The inputs should not be "x_initial" and "x_final". The input
%             % has to be the trajectory (i.e., "planned_lnr_pts_seq").
%             if nargin > 0
%                 obj.filter_mode = filter_mode_inp;
%                 [u_p,obj.kf] = MotionModel_class.compute_planned_control(x_initial,x_final);
%                 x_p = MotionModel_class.compute_planned_traj(x_initial,u_p,obj.kf);
%                 w_p = [zeros(MotionModel_class.wDim,obj.kf),nan(MotionModel_class.wDim,1)];
%                 v_p = [zeros(ObservationModel_class.obsNoiseDim,obj.kf),nan(ObservationModel_class.obsNoiseDim,1)];
%                 
%                 % memory preallocation
%                 obj.planned_lnr_sys_seq = Linear_system_class.empty;
%                 
%                 for k = 1 : obj.kf+1
%                     obj.planned_lnr_pts_seq(k).x = x_p(:,k);
%                     obj.planned_lnr_pts_seq(k).u = u_p(:,k);
%                     obj.planned_lnr_pts_seq(k).w = w_p(:,k);
%                     obj.planned_lnr_pts_seq(k).v = v_p(:,k);
%                     obj.planned_lnr_sys_seq(k) = Linear_system_class(obj.planned_lnr_pts_seq(k));
%                 end
%                 obj.L_seq = LQR_class.time_varying_gains(obj.planned_lnr_sys_seq,obj.kf);
%             end
%         end
        function [next_Hstate, reliable] = propagate_Hstate(obj,old_Hstate,k,noise_mode)
            % propagates the Hstate
            % "k" is the current step, i.e. the time of "old_Hstate". the
            % "next_Hstate" is the Hstate at time "k+1".
            % "noise_mode" must be the last input argument.
            % KG is the Kalman Gain used in the propagation.
            % output "reliable" is 1 if the current estimate is inside the
            % valid linearization region of LQG. It is 0, otherwise.
            wDim = MotionModel_class.wDim;
            VgDim = ObservationModel_class.obsNoiseDim;
            if exist('noise_mode','var') && strcmpi(noise_mode,'No-noise')
                w = zeros(wDim,1);
                Vg = zeros(VgDim,1);
            end
            
            Xg = old_Hstate.Xg;
            b  = old_Hstate.b; % belief
            
            xp = obj.planned_lnr_pts_seq(k).x; % planned x
            % generating feedback controls
%             est_OF_error = b.est_mean.signed_element_wise_dist(xp);  % this basically computes the "signed element-wise distance" between "b.est_mean" and "xp"
            
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
            %%%%%%%%%%%%%%
            disp('<<--Driving difference in q0 to 0 in LQG Class -->>');
            est_OF_error(4)=0;
            reliable = obj.is_in_valid_linearization_region(est_OF_error);
            if ~reliable
                warning('Ali: error is too much; the linearization is not reliable');
            end
            
            feedback_gain = obj.L_seq{k};
            
%             persistent error_hist            
            
            dU = - feedback_gain*est_OF_error; % feedback control du
            
%             error_hist = [error_hist, est_OF_error];
            
            up = obj.planned_lnr_pts_seq(k).u; % planned u
            
            % generating process noises
            if ~exist('noise_mode','var')
                w = MotionModel_class.generate_process_noise(Xg.val,up + dU);
            end
            
            % Hstate propagation
            next_Xg_val = MotionModel_class.f_discrete(Xg.val,up+dU,w);
            
            if next_Xg_val(4) < 0
                disp('q0 < 0 !!!');
                error('q0 went negative in LQG_class propagate');
            end
            
            % generating observation noise
            if ~exist('noise_mode','var')
                Vg = ObservationModel_class.generate_observation_noise(next_Xg_val);
            end
            
            % constructing ground truth observation
            Zg = ObservationModel_class.h_func(next_Xg_val,Vg);
            
            if strcmpi(obj.filter_mode,'LKF')
                b_next = Kalman_filter.LKF_estimate(b,up+dU,Zg,obj.planned_lnr_sys_seq(k),obj.planned_lnr_sys_seq(k+1));
            elseif strcmpi(obj.filter_mode,'EKF')
                b_next = Kalman_filter.EKF_estimate(b,up+dU,Zg);
            end
            
            next_Hstate = Hstate(state(next_Xg_val),b_next);
        end
        function next_Hb_particle = propagate_Hb_particle(obj,old_Hb_particle,k)
            Hparticles = old_Hb_particle.Hparticles;
            % The first particle is the "no-noise" particle.
            if ~old_Hb_particle.stopped_particles(1) && ~old_Hb_particle.collided_particles(1) % We propagate the particles only if it has not been stopped or collided already. Indeed since LQG is a choice for edge-controller Not for node-controller, the stopping check on this line seems unnecessary.
                next_Hparticles(1) = obj.propagate_Hstate(Hparticles(1),k,'No-noise');
            else
                next_Hparticles(1) = Hparticles(1);
            end
            
            for i = 2:old_Hb_particle.num_p
                if ~old_Hb_particle.stopped_particles(i) && ~old_Hb_particle.collided_particles(i) % We propagate the particles only if it has not been stopped or collided already. Indeed since LQG is a choice for edge-controller Not for node-controller, the stopping check on this line seems unnecessary.
                    next_Hparticles(i) = obj.propagate_Hstate(Hparticles(i),k);
                else
                    next_Hparticles(i) = Hparticles(i);
                end
            end
            next_Hb_particle = old_Hb_particle; % We want to maintain the other information in Hbelief and only change its "Hparticles" property, which is changed in the next line.
            next_Hb_particle.Hparticles = next_Hparticles; % in this line we update the "Hparticles" property of the Hbelief_p.
        end
        function next_Hb_Gaussian = propagate_Hb_Gaussian(obj,Hb_G_curr,k)
            % "k" is the current time.
            % Important: Throughout this function "curr" means time step
            % "k" and "next" means time step "k+1".
            BigX_curr = [Hb_G_curr.Xg_mean.val;Hb_G_curr.Xest_mean_mean.val];
            BigCov_curr = Hb_G_curr.P_of_joint;
            Pest_curr = Hb_G_curr.Pest;
            xp_bar_curr = [obj.planned_lnr_pts_seq(k).x;obj.planned_lnr_pts_seq(k).x];
            xp_bar_next = [obj.planned_lnr_pts_seq(k+1).x;obj.planned_lnr_pts_seq(k+1).x];
            
            stDim = MotionModel_class.stDim;
            vDim = ObservationModel_class.obsNoiseDim;
            A_curr = obj.planned_lnr_sys_seq(k).A; % "curr" or "current" means "at time k"
            B_curr = obj.planned_lnr_sys_seq(k).B; % "curr" or "current" means "at time k"
            G_curr = obj.planned_lnr_sys_seq(k).G;
            Q_curr = obj.planned_lnr_sys_seq(k).Q;
            H_next = obj.planned_lnr_sys_seq(k+1).H; % "next" means "at time k+1"
            M_next = obj.planned_lnr_sys_seq(k+1).M;
            R_next = obj.planned_lnr_sys_seq(k+1).R;
            L_curr = obj.L_seq{k};
            Pprd_next = A_curr*Pest_curr*A_curr' + G_curr*Q_curr*G_curr';
            K_next = (Pprd_next*H_next')/(H_next*Pprd_next*H_next'+R_next); %K_next is the "Kalman Gain" which is associated with the time k+1.
            Pest_next = Pprd_next - K_next*H_next*Pprd_next; % we can use Joseph form to ensure the positive definitness of estimation covariance, but it will need another inversion in this line too.

            BigF = [A_curr,-B_curr*L_curr;K_next*H_next*A_curr,A_curr-B_curr*L_curr-K_next*H_next*A_curr];
            BigG = [G_curr,zeros(stDim,vDim);K_next*H_next*G_curr,K_next*M_next];
            BigQ = blkdiag(Q_curr,R_next);
                
            BigX_next = xp_bar_next + BigF*(BigX_curr - xp_bar_curr); % the "curr" and "next" subscripts are extremely important here. If you do not pay attention, you may make a mistake, which is sometime very hard to find out.
            XgMean_next = state(BigX_next(1:stDim));
            Xest_mean_of_mean_next = state(BigX_next(stDim+1:2*stDim));
            BigCov_next = BigF*BigCov_curr*BigF' + BigG*BigQ*BigG';

            next_Hb_Gaussian = Hbelief_G(XgMean_next, Xest_mean_of_mean_next, Pest_next, BigCov_next);
        end
        function obj = erase_from_memory(obj)
            obj.planned_lnr_pts_seq = [];
            obj.planned_lnr_sys_seq = [];
            obj.L_seq = [];
        end
    end
    
    methods (Access = private)
        function YesNo = is_in_valid_linearization_region(obj,est_OF_error)
            YesNo = all(abs(est_OF_error) < obj.valid_lnr_domain); % never forget the "absolute value operator" in computing distances.

        end
    end
end