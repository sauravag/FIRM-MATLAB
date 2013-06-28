classdef Dynamical_planar_manipulator < MotionModel_interface
    properties (Constant)
        num_revolute_joints = user_data_class.par.state_parameters.num_revolute_joints;
        stDim = state.dim; % state dimension
        ctDim = Dynamical_planar_manipulator.num_revolute_joints;  % control vector dimension
        wDim = Dynamical_planar_manipulator.num_revolute_joints;   % Process noise (W) dimension
        dt = user_data_class.par.motion_model_parameters.dt;
        alpha_max = 10;%22.5;% maximum accelaration
        sigma_acceleration = 0.003162; % note that this is a noise on angular velocity and it is equal for all joints
    end
    
    methods (Static)
        %% Discrete dynamics
        % $$ x_{k+1} = x_{k}+ \omega\delta t + w\sqrt{\delta t}$$
        function x_next = f_discrete(x,u,w)
            n = Dynamical_planar_manipulator.num_revolute_joints;
            dt = Dynamical_planar_manipulator.dt;%#ok<PROP>
            x_next = [eye(n) , eye(n)*dt ; zeros(n,n) , eye(n)]*x + [zeros(n) ; eye(n)*dt]*u + [zeros(n) ; eye(n)*sqrt(dt)]*w;%#ok<PROP>
        end
        function A = df_dx_func(x,u,w) %#ok<INUSD>
            n = Dynamical_planar_manipulator.num_revolute_joints;
            dt = Dynamical_planar_manipulator.dt;%#ok<PROP>
            A = [eye(n) , eye(n)*dt ; zeros(n,n) , eye(n)];%#ok<PROP>
        end
        function B = df_du_func(x,u,w) %#ok<INUSD>
            n = Dynamical_planar_manipulator.num_revolute_joints;
            dt = Dynamical_planar_manipulator.dt;%#ok<PROP>
            B = [zeros(n) ; eye(n)*dt];%#ok<PROP>
        end
        function G = df_dw_func(x,u,w) %#ok<INUSD>
            n = Dynamical_planar_manipulator.num_revolute_joints;
            dt = Dynamical_planar_manipulator.dt; %#ok<PROP>
            G = [zeros(n) ; eye(n)*sqrt(dt)];%#ok<PROP>
        end
        function w = generate_process_noise(x,u)
            n = Dynamical_planar_manipulator.wDim;
            w = mvnrnd(zeros(n,1),Dynamical_planar_manipulator.process_noise_cov(x,u))';
        end
        %% Computing the covarinace of process noise in the CONTINUOUS dynamics
        % $$ Q = \sigma_{\omega}^2*I$$
        function Q_process_noise = process_noise_cov(x,u) %#ok<INUSD>
            n = Dynamical_planar_manipulator.wDim;
            Q_process_noise = (Dynamical_planar_manipulator.sigma_acceleration)^2 * eye(n);
        end
        function nominal_traj = generate_VALID_open_loop_point2point_traj(X_initial,X_final)  % generates COLLISION-FREE open-loop trajectories between two start and goal states
            if isa(X_initial,'state'), X_initial=X_initial.val; end % retrieve the value of the state vector
            if isa(X_final,'state'), X_final=X_final.val; end % retrieve the value of the state vector
            n = Dynamical_planar_manipulator.wDim;
            dt=Dynamical_planar_manipulator.dt; %#ok<PROP>
            accel_max = Dynamical_planar_manipulator.alpha_max; % constant rotational velocity during turnings
            
            X_final_state = state(X_final);
            state_signed_diff = X_final_state.signed_element_wise_dist(X_initial);
            half_theta_dist = state_signed_diff(1:n)/2;
            
            abs_th_max = max(abs(half_theta_dist));
            T_min = sqrt(2*abs_th_max/accel_max); % minimum amound of time required
            half_kf = ceil(T_min/dt); %#ok<PROP>
            
            accel = half_theta_dist*(2/(half_kf*dt)^2); %#ok<PROP>
            decel = -accel;
            
            u_p =  [repmat(accel , 1 , half_kf) , repmat(decel , 1 , half_kf)];
            kf = ceil(2*half_kf);

            %=====================Nominal control and state trajectory generation
            x_p = zeros(Dynamical_planar_manipulator.stDim,kf+1);
            x_p(:,1) = X_initial;
            w_zero = zeros(Dynamical_planar_manipulator.wDim , 1);
            for k = 1:kf
                x_p(:,k+1) = Dynamical_planar_manipulator.f_discrete( x_p(:,k) , u_p(:,k) , w_zero);
                tmp = state(x_p(:,k+1)); if tmp.is_constraint_violated, nominal_traj =[]; return; end
            end
            nominal_traj.x = x_p;nominal_traj.u = u_p;
        end
        function YesNo = is_constraints_violated(open_loop_traj) % this function checks if the "open_loop_traj" violates any constraints or not. For example it checks collision with obstacles.
            kf = size(open_loop_traj.u,2);
            YesNo=0; % initialization
            for i = 1:kf+1
                tmp_state = state(open_loop_traj.x(:,i));
                if tmp_state.is_constraint_violated()
                    YesNo = 1;
                    return
                end
            end
        end
        function traj_plot_handle = draw_nominal_traj(nominal_traj, traj_flag) %#ok<INUSD>
            traj_plot_handle = nan(1 , size(nominal_traj.x , 2));
            for k = 1 : size(nominal_traj.x , 2)
                tmp_Xstate = state (nominal_traj.x(:,k) );
                tmp_Xstate.draw();%,'TriaColor',color(cycles));
                % traj_plot_handle(:,k) = tmp_Xstate.plot_handle;
            end
        end
    end
end


