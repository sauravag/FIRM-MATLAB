classdef Controller_base_class
    %   LQG_stationary_class encapsulates the Stationary Linear Quadratic Gaussian Controller
    properties
        Estimator
        Separated_controller
    end
    
    methods (Abstract)
        function [next_Hstate, reliable] = propagate_Hstate(obj,old_Hstate,noise_mode)
            error('not yet')
            % propagates the Hstate using Stationary Kalman Filter and
            % Stationary LQR.
            % "noise_mode" must be the last input argument.
            % output "reliable" is 1 if the current estimate is inside the
            % valid linearization region of LQG. It is 0, otherwise.
            wDim = MotionModel_class.wDim;
            VgDim = ObservationModel_class.obsNoiseDim;
            if exist('noise_mode','var') && strcmpi(noise_mode,'No-noise')
                w = zeros(wDim,1);
                Vg = zeros(VgDim,1);
            end
            
            Xg = old_Hstate.Xg;
            b = old_Hstate.b; % belief

            % generating feedback controls
            [u , reliable] = obj.Stationary_LQR.generate_feedback_control(b);
            
            % generating process noises
            if ~exist('noise_mode','var')
                w = MotionModel_class.generate_process_noise(Xg.val,u);
            end
            
            % Hstate propagation
            next_Xg_val = MotionModel_class.f_discrete(Xg.val,u,w);
            
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
            b_next = Kalman_filter.LKF_estimate(b,u,Zg,obj.lnr_sys,obj.lnr_sys);
            
            next_Hstate = Hstate(state(next_Xg_val),b_next);
        end
        function next_Hb_particle = propagate_Hb_particle(obj,old_Hb_particle)
            error('not yet')
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
    end
    
end