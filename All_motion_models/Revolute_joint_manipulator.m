classdef Revolute_joint_manipulator < MotionModel_interface
    properties (Constant)
        num_revolute_joints = user_data_class.par.state_parameters.num_revolute_joints;
        stDim = state.dim; % state dimension
        ctDim = state.dim;  % control vector dimension
        wDim = state.dim;   % Process noise (W) dimension
        dt = user_data_class.par.motion_model_parameters.dt;
        sigma_angular_velocity = 0.1*pi/180; % note that this is a noise on angular velocity and it is equal for all joints
        omega_max = 45*pi/180;
    end
    
    methods (Static)
        %% Discrete dynamics
        % $$ x_{k+1} = x_{k}+ \omega\delta t + w\sqrt{\delta t}$$
        function x_next = f_discrete(x,u,w)
            x_next = x + u*Revolute_joint_manipulator.dt + w*sqrt(Revolute_joint_manipulator.dt);
        end
        function A = df_dx_func(x,u,w) %#ok<INUSD>
            A = eye(Revolute_joint_manipulator.stDim);
        end
        function B = df_du_func(x,u,w) %#ok<INUSD>
            B = eye(Revolute_joint_manipulator.stDim)*Revolute_joint_manipulator.dt;
        end
        function G = df_dw_func(x,u,w) %#ok<INUSD>
            G = eye(Revolute_joint_manipulator.stDim)*sqrt(Revolute_joint_manipulator.dt);
        end
        function w = generate_process_noise(x,u)
            w = mvnrnd(zeros(Revolute_joint_manipulator.stDim,1),Revolute_joint_manipulator.process_noise_cov(x,u))';
        end
        %% Computing the covarinace of process noise in the CONTINUOUS dynamics
        % $$ Q = \sigma_{\omega}^2*I$$
        function Q_process_noise = process_noise_cov(x,u) %#ok<INUSD>
            Q_process_noise = (Revolute_joint_manipulator.sigma_angular_velocity)^2 * eye(Revolute_joint_manipulator.stDim);
        end
        function nominal_traj = generate_open_loop_point2point_traj(X_initial,X_final) % generates open-loop trajectories between two start and goal states
            disp('This is an obsolete function')
            if isa(X_initial,'state'), X_initial=X_initial.val; end % retrieve the value of the state vector
            if isa(X_final,'state'), X_final=X_final.val; end % retrieve the value of the state vector
            % parameters (same for all team)
            dt=Revolute_joint_manipulator.dt; %#ok<PROP>
            % parameters (possibly different for each robot)
            omega_max = Revolute_joint_manipulator.omega_max; %#ok<PROP> % constant rotational velocity during turnings
            X_final_state = state(X_final);
            th_signed_diff = X_final_state.signed_element_wise_dist(X_initial);
            abs_th_max = max(abs(th_signed_diff));
            kf_rational = abs_th_max/(omega_max*dt); %#ok<PROP>
            kf = ceil(kf_rational);
            omega_rational_max = omega_max*kf_rational/kf; %#ok<PROP> % Consider the joint with maximum theta difference. if you go with "omega_rational_max" you reach the final angle in "kf" steps.
            omega = omega_rational_max*(th_signed_diff/abs_th_max); % computes the angular velocity for all joints such that they reach the final state all at the same time.
            u_p = repmat(omega , 1 , kf);
            
            %=====================Nominal control and state trajectory generation
            x_p = zeros(Revolute_joint_manipulator.stDim,kf+1);
            x_p(:,1) = X_initial;
            w_zero = zeros(Revolute_joint_manipulator.wDim , 1);
            for k = 1:kf
                x_p(:,k+1) = Revolute_joint_manipulator.f_discrete( x_p(:,k) , u_p(:,k) , w_zero);
            end
            nominal_traj.x = x_p;nominal_traj.u = u_p;
        end
        function nominal_traj = generate_VALID_open_loop_point2point_traj(X_initial,X_final)  % generates COLLISION-FREE open-loop trajectories between two start and goal states
            if isa(X_initial,'state'), X_initial=X_initial.val; end % retrieve the value of the state vector
            if isa(X_final,'state'), X_final=X_final.val; end % retrieve the value of the state vector
            % parameters (same for all team)
            dt=Revolute_joint_manipulator.dt; %#ok<PROP>
            % parameters (possibly different for each robot)
            omega_max = Revolute_joint_manipulator.omega_max; %#ok<PROP> % constant rotational velocity during turnings
            X_final_state = state(X_final);
            th_signed_diff = X_final_state.signed_element_wise_dist(X_initial);
            abs_th_max = max(abs(th_signed_diff));
            kf_rational = abs_th_max/(omega_max*dt); %#ok<PROP>
            kf = ceil(kf_rational);
            omega_rational_max = omega_max*kf_rational/kf; %#ok<PROP> % Consider the joint with maximum theta difference. if you go with "omega_rational_max" you reach the final angle in "kf" steps.
            omega = omega_rational_max*(th_signed_diff/abs_th_max); % computes the angular velocity for all joints such that they reach the final state all at the same time.
            u_p = repmat(omega , 1 , kf);
            
            %=====================Nominal control and state trajectory generation
            x_p = zeros(Revolute_joint_manipulator.stDim,kf+1);
            x_p(:,1) = X_initial;
            w_zero = zeros(Revolute_joint_manipulator.wDim , 1);
            for k = 1:kf
                x_p(:,k+1) = Revolute_joint_manipulator.f_discrete( x_p(:,k) , u_p(:,k) , w_zero);
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
        function traj_plot_handle = draw_nominal_traj(nominal_traj, traj_flag)
            traj_plot_handle = [];
            if traj_flag == 1
                traj_plot_handle = nan(1 , kf+1);
                for k = 1 : size(nominal_traj.x , 2)
                    tmp_Xstate = state (nominal_traj.x(:,k) );
                    tmp_Xstate.draw('RobotShape','triangle','robotsize',1);%,'TriaColor',color(cycles));
                    traj_plot_handle(:,i) = tmp_state.plot_handle;
                end
            else
                tmp_handle = plot(nominal_traj.x(1,:) , nominal_traj.x(2,:));
                traj_plot_handle = [traj_plot_handle , tmp_handle];
           end
        end
    end
end


