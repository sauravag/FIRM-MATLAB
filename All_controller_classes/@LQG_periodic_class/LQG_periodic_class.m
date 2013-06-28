classdef LQG_periodic_class < LQG_base_class
    %   LQG_periodic_class encapsulates the Periodic Linear Quadratic Gaussian Controller
    properties (Constant = true)
        valid_lnr_domain = user_data_class.par.valid_linearization_domain;
    end
    properties
        T   % period
        lnr_pts_periodic;  % This is called "orbit" in Ali's writings.
        lnr_sys_periodic;
        periodic_belief; % This is actually the "mean" of periodic belief. % This may be removed in future as it is a included already in "Periodic_Gaussian_Hb".
        Periodic_Gaussian_Hb;
        Periodic_Kalman_gain;
        Periodic_Feedback_gain;
        Periodic_Big_lnr_sys;
        Periodic_Pest;
    end
    
    methods
        function obj = LQG_periodic_class(orbit)
            % note that the orbit is a structure with two fields: periodic state trajectory and periodic control trajectory
            obj.T = size(orbit.x,2);  % retrieve the period % note tha the length of orbit is "T"
            w_zero = zeros(MotionModel_class.wDim,1);
            v_zero = zeros(ObservationModel_class.obsNoiseDim,1);
           
            % memory preallocation
            obj.lnr_sys_periodic = Linear_system_class.empty;
            for k = 1 : obj.T
                 obj.lnr_pts_periodic(k).x = orbit.x(:,k);
                 obj.lnr_pts_periodic(k).u = orbit.u(:,k);
                 obj.lnr_pts_periodic(k).w = w_zero;
                obj.lnr_pts_periodic(k).v = v_zero;
                obj.lnr_sys_periodic(k) = Linear_system_class(obj.lnr_pts_periodic(k));
            end
            
            [K_periodic,~,Pest_periodic] = Kalman_filter.periodic_gain_and_covariances(obj.lnr_sys_periodic); % K_periodic is the periodic Kalman filter gain.
            obj.Periodic_Kalman_gain = K_periodic;
            obj.Periodic_Pest = Pest_periodic;
            L_periodic = LQR_class.periodic_gains(obj.lnr_sys_periodic); % Lss is the stationary feedback gain.
            obj.Periodic_Feedback_gain = L_periodic;
        end
        function Big_system_val = get.Periodic_Big_lnr_sys(obj)
            error('This function has not been changed from stationary to periodic yet')
            % The property "Big_lnr_system" is computed only the
            % first time it is needed.
            if isempty(obj.Big_lnr_sys)
                stDim = state.dim;
                obsNoiseDim = ObservationModel_class.obsNoiseDim;
                
                An = obj.lnr_sys.A;
                Bn = obj.lnr_sys.B;
                Gn = obj.lnr_sys.G;
                Qn = obj.lnr_sys.Q;
                Hn = obj.lnr_sys.H;
                Mn = obj.lnr_sys.M;
                Rn = obj.lnr_sys.R;
                Ln = obj.Stationary_Feedback_gain;
                Kn = obj.Stationary_Kalman_gain;
                % Following lines changes the "obj" because the class is a subclass of the "handle" class.
                % Otherwise, we had to output the "obj".
                obj.Big_lnr_sys.BigF = [An,-Bn*Ln;Kn*Hn*An,An-Bn*Ln-Kn*Hn*An];
                obj.Big_lnr_sys.BigG = [Gn,zeros(stDim,obsNoiseDim);Kn*Hn*Gn,Kn*Mn];
                obj.Big_lnr_sys.BigQ = blkdiag(Qn,Rn);
                Big_system_val = obj.Big_lnr_sys;
            else
                Big_system_val = obj.Big_lnr_sys;
            end
        end
        function stationGHb_val = get.Periodic_Gaussian_Hb(obj)
            error('This function has not been changed from stationary to periodic yet')
            % The property "Stationary_Gaussian_Hb" is computed only the
            % first time it is needed.
            if isempty(obj.Stationary_Gaussian_Hb)
                stationGHb_val = obj.stationary_Gaussian_Hbelief(); % here, we call a private function of this class.
                obj.Stationary_Gaussian_Hb = stationGHb_val; % This line works because the class is a subclass of the "handle" class. Otherwise, we had to output the "obj".
            else
                stationGHb_val = obj.Stationary_Gaussian_Hb;
            end
        end
        function periodic_belief_val = get.periodic_belief(obj)
            % The property "periodic_belief" is computed only once, the
            % first time it is needed.
            if isempty(obj.periodic_belief)
                periodic_belief_val = belief.empty;
                for k = 1 : obj.T
                    periodic_belief_val(k) = belief( state(obj.lnr_pts_periodic(k).x) , obj.Periodic_Pest(:,:,k) );
                end
                obj.periodic_belief = periodic_belief_val; % This line works because the class is a subclass of the "handle" class. Otherwise, we had to output the "obj".
            else
                periodic_belief_val = obj.periodic_belief;
            end
        end
        function [next_Hstate, reliable] = propagate_Hstate(obj,old_Hstate,k,noise_mode)
            % propagates the Hstate using Periodic Kalman Filter and
            % Periodic LQR.
            % "noise_mode" must be the last input argument.
            % output "reliable" is 1 if the current estimate is inside the
            % valid linearization region of LQG. It is 0, otherwise.
            
            k = mod(k,obj.T); % This line is a crucial line, and makes the time periodic by the period "T".
            if k==0, k = obj.T; end
            
            wDim = MotionModel_class.wDim;
            VgDim = ObservationModel_class.obsNoiseDim;
            if exist('noise_mode','var') && strcmpi(noise_mode,'No-noise')
                w = zeros(wDim,1);
                Vg = zeros(VgDim,1);
            end
            
            Xg = old_Hstate.Xg;
            b = old_Hstate.b; % belief
            xp = obj.lnr_pts_periodic(k).x; % planned x (or target point) or linearization point.
            % generating feedback controls
            est_OF_error = b.est_mean.signed_element_wise_dist(xp);  % this basically computes the "signed element-wise distance" between "b.est_mean" and "xp"
            
            reliable = obj.is_in_valid_linearization_region(est_OF_error);
            if ~reliable
                warning('Ali: error is too much; the linearization is not reliable');
            end
            feedback_gain = obj.Periodic_Feedback_gain(:,:,k);
            dU = - feedback_gain*est_OF_error;
            up = obj.lnr_pts_periodic(k).u; % planned up, which usually (or maybe always) must be zero in stationary LQG setting.
            
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
            
            % Since "PeriodicKF_estimate" leads to unsymmetric estimation
            % covariances, we use the LKF instead of
            % b_next = Kalman_filter.PeriodicKF_estimate(b,up+dU,Zg,obj.lnr_sys_periodic(k),obj.lnr_sys_periodic(k+1),obj.Periodic_Kalman_gain(k+1));
            
            next_k = k+1;
            next_k = mod(next_k,obj.T); % This line is a crucial line, and makes the time periodic by the period "T".
            if next_k==0, next_k = obj.T; end
            b_next = Kalman_filter.LKF_estimate(b,up+dU,Zg,obj.lnr_sys_periodic(k),obj.lnr_sys_periodic(next_k)); 
            
            next_Hstate = Hstate(state(next_Xg_val),b_next);
        end
        function next_Hb_particle = propagate_Hb_particle(obj,old_Hb_particle,time_step)
            Hparticles = old_Hb_particle.Hparticles;
            % The first particle is the "no-noise" particle.
            if ~old_Hb_particle.collided_particles(1) && ~old_Hb_particle.stopped_particles(1) % We propagate the first particle only if it has not been stopped already and it has not been collided yet.
                next_Hparticles(1) = obj.propagate_Hstate(Hparticles(1),time_step,'No-noise');
            else
                next_Hparticles(1) = Hparticles(1);
            end
            
            for i = 2:old_Hb_particle.num_p
                if ~old_Hb_particle.collided_particles(i) && ~old_Hb_particle.stopped_particles(i) % We propagate the i-th particle only if it has not been stopped already and it has not been collided yet.
                    next_Hparticles(i) = obj.propagate_Hstate(Hparticles(i),time_step); % with noise (i.e. noise is not zero for the rest of particles.)
                else
                    next_Hparticles(i) = Hparticles(i);
                end
            end
            next_Hb_particle = old_Hb_particle; % We want to maintain the other information in Hbelief and only change its "Hparticles" property, which is changed in the next line.
            next_Hb_particle.Hparticles = next_Hparticles; % in this line we update the "Hparticles" property of the Hbelief_p.
        end
        function next_Hb_Gaussian = propagate_Hb_Gaussian(obj,old_Hb_G)
            error('This function has not been changed from stationary to periodic yet')
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
            error('This function has not been changed from stationary to periodic yet')
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
    end
end