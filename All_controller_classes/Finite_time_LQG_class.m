classdef Finite_time_LQG_class < LQG_interface
    % FTLQG (Finite Time LQG) encapsulates the finite time-varying Linear Quadratic Gaussian Controller
    % Note that this class can use a lot of memory depending on the length
    % of state trajectory. Thus, you should clear the objects of this class
    % when you do not need them anymore. You can do that using
    % "erase_from_memory" function.
    
    properties
        estimator
        separated_controller
        lnr_pts;
        lnr_sys;
        kf;
        filter_mode; % this can be "EKF" or "LKF"
        L_seq;
    end
    
    methods
        function obj = Finite_time_LQG_class(nominal_trajectory, filter_mode_inp)
            if nargin < 2, filter_mode_inp = 'LKF'; end  % the default filter in Finite_time_LQG is LKF (linearized Kalman filter)
            if nargin > 0
                obj.filter_mode = filter_mode_inp;
                obj.kf = size(nominal_trajectory.x , 2)-1;
                
                x_p = nominal_trajectory.x;
                u_p = nominal_trajectory.u;
                if size(u_p,2)<obj.kf+1 
                    u_p = [u_p,nan(MotionModel_class.ctDim,1)]; 
                end
                w_zero = MotionModel_class.zeroNoise;
                v_zero = ObservationModel_class.zeroNoise;
                
                % memory preallocation
                obj.lnr_sys = Linear_system_class.empty;
                
                for k = 1 : obj.kf+1
                    obj.lnr_pts(k).x = x_p(:,k);
                    obj.lnr_pts(k).u = u_p(:,k);
                    obj.lnr_pts(k).w = w_zero;
                    obj.lnr_pts(k).v = v_zero;
                    obj.lnr_sys(k) = Linear_system_class(obj.lnr_pts(k));
                end
                 
                if strcmpi(obj.filter_mode, 'LKF')
                    obj.estimator = LKF;
                elseif strcmpi(obj.filter_mode, 'EKF')
                    obj.estimator = EKF(obj.lnr_sys);
                else
                    error(['FiniteTime LQG does not support ', obj.filter_mode, ' as a filter'])
                end
                
                obj.separated_controller = Finite_time_LQR_class(obj.lnr_sys, obj.lnr_pts);
                
                obj.L_seq = obj.separated_controller.Feedback_gains;
                
            end
        end
        function [next_Hstate, reliable] = propagate_Hstate(obj,old_Hstate,k,noise_mode)
            % propagates the Hstate
            % "k" is the current step, i.e. the time of "old_Hstate". the
            % "next_Hstate" is the Hstate at time "k+1".
            % "noise_mode" must be the last input argument.
            % output "reliable" is 1 if the current estimate is inside the
            % valid linearization region of LQG. It is 0, otherwise.
            
            if exist('noise_mode','var') && strcmpi(noise_mode,'No-noise')
                w = MotionModel_class.zeroNoise;
                Vg = ObservationModel_class.zeroNoise;
            end
            
            Xg = old_Hstate.Xg;
            b  = old_Hstate.b; % belief
            
            % generating feedback controls
            [u , reliable] = obj.separated_controller.generate_feedback_control(b,k);
            if ~reliable, warning('Controller_class: Error is too much; the linearization is not reliable'); end %#ok<WNTAG>
            
            % generating process noises
            if ~exist('noise_mode','var')
                w = MotionModel_class.generate_process_noise(Xg.val,u);
            end
            
            % True state propagation
            next_Xg_val = MotionModel_class.f_discrete(Xg.val,u,w);
            % generating observation noise
            if ~exist('noise_mode','var')
                Vg = ObservationModel_class.generate_observation_noise(next_Xg_val);
            end
            
            % constructing ground truth observation
            Zg = ObservationModel_class.h_func(next_Xg_val,Vg);
            
            if strcmpi(obj.filter_mode,'LKF')
                b_next = obj.estimator.estimate(b, u, Zg, obj.lnr_sys(k), obj.lnr_sys(k+1));
            elseif strcmpi(obj.filter_mode,'EKF')
                b_next = obj.estimator.estimate(b, u, Zg);
            else
                error(['FiniteTime LQG does not support ', obj.filter_mode, ' as a filter'])
            end
            
            next_Hstate = Hstate(state(next_Xg_val),b_next);
        end
        function [nextBelief, reliable,sim] = executeOneStep(obj,oldBelief,sim,k,noiseFlag)
            % propagates the system and belief using Linearized/Extended Kalman Filter and
            % Finite-time LQR.
            % "noiseFlag" must be the last input argument.
            % output "reliable" is 1 if the current estimate is inside the
            % valid linearization region of LQG. It is 0, otherwise.

            % generating feedback controls
            [u , reliable] = obj.separated_controller.generate_feedback_control(oldBelief,k);
            % Apply control
            sim = sim.evolve(u, noiseFlag);
            % sim = sim.refresh();
            
            % Get observation from simulator
            z = sim.getObservation(noiseFlag);
            
            % Estimation procedure
             if strcmpi(obj.filter_mode,'LKF')
                nextBelief = obj.estimator.estimate(oldBelief, u, z, obj.lnr_sys(k), obj.lnr_sys(k+1));
            elseif strcmpi(obj.filter_mode,'EKF')
                nextBelief = obj.estimator.estimate(oldBelief, u, z);
            else
                error(['FiniteTime LQG does not support ', obj.filter_mode, ' as a filter'])
             end
            
        end
        
        function next_Hb_particle = propagate_Hb_particle(obj,old_Hb_particle,k)
            Hparticles = old_Hb_particle.Hparticles;
            % The first particle is the "no-noise" particle.
            disp(['step number ',num2str(k),';  particle  ',num2str(1)])
%             disp(['just for a test !!! Change it now!!!!!!!!!!!!!!!!!!!1  Add no-noise'])
            if ~old_Hb_particle.stopped_particles(1) && ~old_Hb_particle.collided_particles(1) % We propagate the particles only if it has not been stopped or collided already. Indeed since LQG is a choice for edge-controller Not for node-controller, the stopping check on this line seems unnecessary.
                next_Hparticles(1) = obj.propagate_Hstate(Hparticles(1),k,'No-noise');
%                 next_Hparticles(1) = obj.propagate_Hstate(Hparticles(1),k);
            else
                next_Hparticles(1) = Hparticles(1);
            end
            
            for i = 2:old_Hb_particle.num_p
                disp(['step number ',num2str(k),';  particle  ',num2str(i)])
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
%             error('This function has not been updated')
            % "k" is the current time.
            % Important: Throughout this function "curr" means time step
            % "k" and "next" means time step "k+1".
            BigX_curr = [Hb_G_curr.Xg_mean.val;Hb_G_curr.Xest_mean_mean.val];
            BigCov_curr = Hb_G_curr.P_of_joint;
            Pest_curr = Hb_G_curr.Pest;
            xp_bar_curr = [obj.lnr_pts(k).x;obj.lnr_pts(k).x];
            xp_bar_next = [obj.lnr_pts(k+1).x;obj.lnr_pts(k+1).x];
            
            stDim = MotionModel_class.stDim;
            vDim = ObservationModel_class.obsNoiseDim;
            A_curr = obj.lnr_sys(k).A; % "curr" or "current" means "at time k"
            B_curr = obj.lnr_sys(k).B; % "curr" or "current" means "at time k"
            G_curr = obj.lnr_sys(k).G;
            Q_curr = obj.lnr_sys(k).Q;
            H_next = obj.lnr_sys(k+1).H; % "next" means "at time k+1"
            M_next = obj.lnr_sys(k+1).M;
            R_next = obj.lnr_sys(k+1).R;
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
            obj.lnr_pts = [];
            obj.lnr_sys = [];
            obj.L_seq = [];
        end
        function nextHyperBelief = propagateHyperBelief(obj,oldHyperBelief)
            error('not yet implemented');
        end
        
    end
    
end