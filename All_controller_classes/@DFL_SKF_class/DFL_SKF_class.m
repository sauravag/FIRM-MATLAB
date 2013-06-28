classdef DFL_SKF_class< handle
    %   nonlinear_unicycle_controller_class encapsulates the nonlinear
    %   controller desinged by Oriolo et al. in [?] for unicycle along with
    %   Stationary KF as an estimator
    properties (Constant = true)
        valid_lnr_domain = user_data_class.par.valid_linearization_domain; % linearizatin domain is still needed, due to the usage of Stationary KF.
    end
    properties
        lnr_pts;  % linearization points for the Stationary KF design
        lnr_sys;  % linear system for Stationary KF 
        Stationary_Kalman_gain; % not applicable
        % Stationary_Feedback_gain; % not applicable
        Stationary_Pest; 
    end
    
    methods
        %% constructor
        function obj = DFL_SKF_class(node)
            % constructor
            % note that the node is not of the type "state". It is only a
            % vector.
            obj.lnr_pts.x = node;
            obj.lnr_pts.u = zeros(MotionModel_class.ctDim,1);
            obj.lnr_pts.w = zeros(MotionModel_class.wDim,1);
            obj.lnr_pts.v = zeros(ObservationModel_class.obsNoiseDim,1);
            obj.lnr_sys = Linear_system_class(obj.lnr_pts);
            [Kss,~,Pest_ss] = Kalman_filter.stationary_gain_and_covariances(obj.lnr_sys); % Kss is the stationary Kalman filter gain.
            obj.Stationary_Kalman_gain = Kss;
            obj.Stationary_Pest = Pest_ss;
        end
        function [next_Hstate, reliable] = propagate_Hstate(obj , old_Hstate , noise_mode)  % since this controller is a "Dynamic feedback linearization-based" controller, it needs the control signal in the previous step to generate the control signal in the current step.
            % propagates the Hstate using Stationary Kalman Filter and
            % nonlinear controller desinged by Oriolo et al. in [?] for unicycle.
            %
            % "noise_mode" must be the last input argument.
            % output "reliable" is 1 if the current estimate is inside the
            % valid linearization region of LQG. It is 0, otherwise.
            
            global Previous_control; % Since we are using a "Dynamic feedback linearization based" controller, at each step
            % we need to have the previous control signal value. The
            % variable "Previous_control" gets initialized to zero every 
            % time we are starting a new edge.
            
            wDim = MotionModel_class.wDim;
            VgDim = ObservationModel_class.obsNoiseDim;
            if exist('noise_mode','var') && strcmpi(noise_mode,'No-noise')
                w = zeros(wDim,1);
                Vg = zeros(VgDim,1);
            end
            
            Xg = old_Hstate.Xg;
            b = old_Hstate.b; % belief
            xp = obj.lnr_pts.x; % planned x (or target point) or linearization point.
            % generating feedback controls
            est_OF_error = b.est_mean.signed_element_wise_dist(xp);  % this basically computes the "signed element-wise distance" between "b.est_mean" and "xp"
            reliable = obj.is_in_valid_linearization_region(est_OF_error);
            dU = obj.Controller_dynamic_feedback_linearization(est_OF_error, Previous_control);
            up = obj.lnr_pts.u; % planned up, which usually (or maybe always) must be zero in stabilizing controllers.
            
            % generating process noises
            if ~exist('noise_mode','var')
                w = MotionModel_class.generate_process_noise(Xg.val,up + dU);
            end
            
            % Hstate propagation
            next_Xg_val = MotionModel_class.f_discrete(Xg.val,up+dU,w);
            
            % generating observation noise
            if ~exist('noise_mode','var')
                Vg = ObservationModel_class.generate_observation_noise(next_Xg_val);
            end
            
            % constructing ground truth observation
            Zg = ObservationModel_class.h_func(next_Xg_val,Vg);
            
            % Since "StationaryKF_estimate" leads to unsymmetric estimation
            % covariances, we use the LKF instead.
            % b_next = Kalman_filter.StationaryKF_estimate(b,up+dU,Zg,obj.lnr_sys,obj.Stationary_Kalman_gain);
            
            % Note that if we use LKF in "Stationary_LQG" the linear system
            % used for "prediction" is the same as the linear system used
            % for "update".
            b_next = Kalman_filter.LKF_estimate(b,up+dU,Zg,obj.lnr_sys,obj.lnr_sys);
            
            Previous_control = dU;
            next_Hstate = Hstate(state(next_Xg_val),b_next);
        end
        function next_Hb_particle = propagate_Hb_particle(obj,old_Hb_particle)
            Hparticles = old_Hb_particle.Hparticles;
            % The first particle is the "no-noise" particle.
            if ~old_Hb_particle.collided_particles(1) && ~old_Hb_particle.stopped_particles(1) % We propagate the first particle only if it has not been stopped already and it has not been collided yet.
                next_Hparticles(1) = obj.propagate_Hstate(Hparticles(1),'No-noise');
            else
                next_Hparticles(1) = Hparticles(1);
            end
            
            for i = 2:old_Hb_particle.num_p
                if ~old_Hb_particle.collided_particles(i) && ~old_Hb_particle.stopped_particles(i) % We propagate the i-th particle only if it has not been stopped already and it has not been collided yet.
                    next_Hparticles(i) = obj.propagate_Hstate(Hparticles(i)); % with noise (i.e. noise is not zero for the rest of particles.)
                else
                    next_Hparticles(i) = Hparticles(i);
                end
            end
            next_Hb_particle = old_Hb_particle; % We want to maintain the other information in Hbelief and only change its "Hparticles" property, which is changed in the next line.
            next_Hb_particle.Hparticles = next_Hparticles; % in this line we update the "Hparticles" property of the Hbelief_p.
        end
        function next_Hb_Gaussian = propagate_Hb_Gaussian(obj,old_Hb_G)
            
            BigX = [old_Hb_G.Xg_mean.val;old_Hb_G.Xest_mean_mean.val];
            BigCov = old_Hb_G.P_of_joint;
            Pest = old_Hb_G.Pest;
            nbar = [obj.lnr_pts.x;obj.lnr_pts.x];
            
            stDim = MotionModel_class.stDim;
            A = obj.lnr_sys.A;
            G = obj.lnr_sys.G;
            Q = obj.lnr_sys.Q;
            H = obj.lnr_sys.H;
            R = obj.lnr_sys.R;
            
            BigF = obj.Big_lnr_sys.BigF;
            BigG = obj.Big_lnr_sys.BigG;
            BigQ = obj.Big_lnr_sys.BigQ;
            
            BigX = nbar + BigF*(BigX - nbar);
            XgMean_next = state(BigX(1:stDim));
            Xest_mean_of_mean_next = state(BigX(stDim+1:2*stDim));
            Pprd_next = A*Pest*A'+G*Q*G';
            % Note that if we use "stationary_KF" in the node controller,
            % the Kalman gain is not computed based on the current
            % prediction covariance and it is computed based on the
            % stationary prediction covariance. Thus, following line would
            % not be correct in that case. However, since we are using the
            % LKF in the "node-controller" (to have symmetric estimation
            % covariances), following line makes sense.
            Pest_next = Pprd_next-Pprd_next*H'*inv(H*Pprd_next*H'+R)*H*Pprd_next; %#ok<MINV> % we use Joseph form to ensure the positive definitness of estimation covariance.
            BigCov_next = BigF*BigCov*BigF' + BigG*BigQ*BigG';
            
            next_Hb_Gaussian = Hbelief_G(XgMean_next, Xest_mean_of_mean_next, Pest_next, BigCov_next);
            
        end
    end
    
    methods (Access = private)
        function stGHb = stationary_Gaussian_Hbelief(obj)
            
            %%% First Formulation
            BigF = obj.Big_lnr_sys.BigF;
            BigG = obj.Big_lnr_sys.BigG;
            BigQ = obj.Big_lnr_sys.BigQ;
            %     BigCov=eye(2*stDim);
            %     for NNN=1:10000
            %         BigCov=BigF*BigCov*BigF'+BigG*BigQ*BigG';
            %     end
            BigCov_better = dlyap(BigF,BigG*BigQ*BigG');
            
            %%% Second Formulation, which is not used here
            %     GreatF=[An-Bn*Ln,-Bn*Ln;zeros(stDim,stDim),An-Kn*Hn*An];
            %     GreatG=[Vn,zeros(stDim,obsDim);Kn*Hn*Vn-Vn,Kn*Wn];
            %     GreatQ=blkdiag(Qn,Rn);
            %
            % %     GreatCov=eye(2*stDim);
            % %     for MMM=1:10000
            % %         GreatCov=GreatF*GreatCov*GreatF'+GreatG*GreatQ*GreatG';
            % %     end
            %
            %     GreatCov_better(:,:,jn)=dlyap(GreatF,GreatG*GreatQ*GreatG');
            
            %%% Due to round off errors, Covariances may not be symmetric anymore
            if max(max(abs(BigCov_better-BigCov_better')))<1e-12
                BigCov_better=(BigCov_better+BigCov_better')/2;
            else
                error('Ali: Covariances are too unsymmetric!!')
            end
            %     if max(max(abs(GreatCov_better(:,:,jn)-GreatCov_better(:,:,jn)')))<1e-12
            %         GreatCov_better(:,:,jn)=(GreatCov_better(:,:,jn)+GreatCov_better(:,:,jn)')/2;
            %     else
            %         error('Ali: Covariances are too unsymmetric!!')
            %     end
            Pest_ss = obj.Stationary_Pest;
            Xg_mean = state(obj.lnr_pts.x);
            Xest_MeanOfMean = Xg_mean; % in the stationary Hbelief, the mean of Xg and the mean of Xest_mean are equal.
            stGHb = Hbelief_G(Xg_mean, Xest_MeanOfMean, Pest_ss,BigCov_better);
        end
        function YesNo = is_in_valid_linearization_region(obj,est_OF_error)
            YesNo = all(abs(est_OF_error) < obj.valid_lnr_domain); % never forget the "absolute value operator" in computing distances.
        end
        %% Discretized version of Equation 8 at page 838 of Oriolo's paper
        %
        % $$ V_{k+1} = V_{k} + \dot{V}_k \delta t = V_{k} + (u_{1_k}\cos\theta_k +
        % u_{2_k}\sin\theta_k) \delta t $$
        %
        % $$ \omega_{k+1} = \frac{u_{2_k}\cos\theta_k - u_{1_k}\sin\theta_k}{V_k}$$
        %
        function dU = Controller_dynamic_feedback_linearization(obj, est_OF_error, last_dU)
            v_old = last_dU(1);
            w_old = last_dU(2);  % omega: angular velocity
            x_old = est_OF_error(1);
            y_old = est_OF_error(2);
            th_old = est_OF_error(3);
            dt = MotionModel_class.dt;
            
            xd_old = v_old*cos(th_old);
            yd_old = v_old*sin(th_old);
            thd_old = w_old;
            
            kp1 = 2;kp2 = 12;kd1 = 3;kd2 = 7;
            u1 = -kp1*x_old -kd1*xd_old;
            u2 = -kp2*y_old -kd2*yd_old;
            
            v = v_old + (u1*cos(th_old) + u2*sin(th_old))*dt;
            if v_old ~= 0
                w = (u2*cos(th_old) - u1*sin(th_old))/v_old;
            else
                w = 0;  % I put it, without no reason!!!! 
            end
            
            vmax = 0.3;wmax = 0.5;
            v = (v<-vmax)*(-vmax) + (v>vmax)*(vmax) + (v >= -vmax && v <= vmax)*v;  % saturating velocity v by vmax and -vmax
            w = (w<-wmax)*(-wmax) + (w>wmax)*(wmax) + (w >= -wmax && w <= wmax)*w; % saturating angular velocity w by wmax and -wmax
            dU = [v;w];
        end
    end
end