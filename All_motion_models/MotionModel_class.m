classdef MotionModel_class < MotionModel_interface
<<<<<<< HEAD
    % Note that because the class is defined as a handle class, the
    % properties must be defined such that they are do not change from an
    % object to another one.
    properties (Constant)
        stDim = state.dim; % state dimension
        ctDim = 4;  % control vector dimension
        wDim = 7;   % Process noise (W) dimension
        zeroControl = zeros(MotionModel_class.ctDim,1);
        zeroNoise = zeros(MotionModel_class.wDim,1);
        dt = user_data_class.par.motion_model_parameters.dt;
        l_1 = user_data_class.par.motion_model_parameters.distBetweenFrontWheels; % the distance between front wheels in cm
        l_2 =  user_data_class.par.motion_model_parameters.distBetweenFrontAndBackWheels; % the distance between front and back wheels in the same side in cm

        sigma_b_u = user_data_class.par.motion_model_parameters.sigma_b_u_KukaBase;
        eta_u = user_data_class.par.motion_model_parameters.eta_u_KukaBase;
        P_Wg = user_data_class.par.motion_model_parameters.P_Wg;
        
    end
    %     properties (Constant = true, SetAccess = private)
    %         UnDim = 3;
    %         WgDim = 3;
    %     end
    
    methods (Static)
=======
    properties (Constant = true)
        stDim = 7;%state.dim; % state dimension
        ctDim = 4;  % control vector dimension
        wDim = 4;   % Process noise (W) dimension  % For the generality we also consider the additive noise on kinematics equation (3 dimension), but it most probably will set to zero. The main noise is a 2 dimensional noise which is added to the controls.
        dt = user_data_class.par.motion_model_parameters.dt;
        % base_length = user_data_class.par.motion_model_parameters.base_length;  % distance between robot's rear wheels.
        sigma_b_u = [0.01 ; 0.001 ; 0.001 ; 0.001];%user_data_class.par.motion_model_parameters.sigma_b_u_aircraft ;
        eta_u = [0.01 ; 0.001 ; 0.001 ; 0.001];%user_data_class.par.motion_model_parameters.eta_u_aircraft ;
        P_Wg = user_data_class.par.motion_model_parameters.P_Wg;
        Max_Roll_Rate = deg2rad(45); % try 45
        Max_Pitch_Rate = deg2rad(45);% try 45
        Max_Yaw_Rate = deg2rad(45);% try 45
        Max_Velocity = 1.0; % m/s
        Min_Velocity = 0.25;% m/s
        zeroNoise = zeros(MotionModel_class.wDim,1);
       turn_radius_min = MotionModel_class.Min_Velocity/MotionModel_class.Max_Yaw_Rate; % indeed we need to define the minimum linear velocity in turnings (on orbits) and then find the minimum radius accordingly. But, we picked the more intuitive way.

    end
    
    %% Methods
    %   methods (Access = private)  %used by this class only
    %         function transition_quat = f_transquat(dt , u ,w) % to calculate transition quaternion
    %             u_res = u + w; % noisy input
    %             u_bar = Quaternion(u_res);
    %             u_mod = norm(u_bar);%making a normalized quaternion
    %             q_scalar = cos((u_mod * dt)/2);
    %             q_common = sin((u_mod * dt)/2)/u_mod;
    %             q_vector = u_res * q_common;
    %             transition_quat = Quaternion(q_scalar,q_vector);%returning transition quaternion
    %         end
    %     end
    
    methods (Static = true)

>>>>>>> ba6fc21e4458b0b1914888aaba4114631ed30a9f
        function x_next = f_discrete(x,u,w)
            Un = w(1:MotionModel_class.ctDim); % The size of Un may be different from ctDim in some other model.
            Wg = w(MotionModel_class.ctDim+1 : MotionModel_class.wDim); % The size of Wg may be different from stDim in some other model.
            Wc = MotionModel_class.f_contin(x,Un,Wg);
            x_next = x+MotionModel_class.f_contin(x,u,0)*MotionModel_class.dt+Wc*sqrt(MotionModel_class.dt);
        end
        function x_dot = f_contin(x,u,wg) % Do not call this function from outside of this class!! % The last input in this method should be w, instead of wg. But, since it is a only used in this class, it does not matter so much.
            l1 = MotionModel_class.l1;
            l2 = MotionModel_class.l2;
            gama1= 2/(l1+l2);
            x_dot = (1/4)*[1      -1     -1      1;...
                           1       1      1      1;...
                           -gmam1 gama1 -gama1 gama1]*u+wg;
        end
        function A = df_dx_func(x,u,w)
            un = w(1:MotionModel_class.ctDim); % The size of Un may be different from ctDim in some other model.
            wg = w(MotionModel_class.ctDim+1 : MotionModel_class.wDim); % The size of Wg may be different from stDim in some other model.
            A = eye(MotionModel_class.stDim) ...
                + MotionModel_class.df_contin_dx(x,u,zeros(MotionModel_class.stDim,1))*MotionModel_class.dt ...
                + MotionModel_class.df_contin_dx(x,un,wg)*sqrt(MotionModel_class.dt);
        end
        function Acontin = df_contin_dx(x,u,w) %#ok<INUSD>
            Acontin = zeros(3,3);
        end
        function B = df_du_func(x,u,w) %#ok<INUSD>
            l1 = MotionModel_class.l1;
            l2 = MotionModel_class.l2;
            gama1= 2/(l1+l2);
            B     = (1/4)*[1      -1     -1      1;...
                           1       1      1      1;...
                           -gmam1 gama1 -gama1 gama1]*MotionModel_class.dt;
            end
        function G = df_dw_func(x,u,w) %#ok<INUSD>
            B     = (1/4)*[1      -1     -1      1;...
                           1       1      1      1;...
                           -gmam1 gama1 -gama1 gama1]*MotionModel_class.dt;
            G = [B,eye(MotionModel_class.stDim)]*sqrt(MotionModel_class.dt);
        end
        function w = generate_process_noise(x,u) %#ok<INUSD>
            [Un,Wg] = generate_control_and_indep_process_noise(u);
            w = [Un;Wg];
        end
        function Q_process_noise = process_noise_cov(x,u) %#ok<INUSD>
            P_Un = control_noise_covariance(u);
            Q_process_noise = blkdiag(P_Un,MotionModel_class.P_Wg);
        end
        function nominal_traj = generate_open_loop_point2point_traj(X_initial,X_final) % generates open-loop trajectories between two start and goal states
            if isa(X_initial,'state'), X_initial=X_initial.val; end % retrieve the value of the state vector
            if isa(X_final,'state'), X_final=X_final.val; end % retrieve the value of the state vector
            % parameters
            omega_path=user_data_class.par.motion_model_parameters.omega_const_path; % constant rotational velocity during turnings
            dt=MotionModel_class.dt;
            V_path=user_data_class.par.motion_model_parameters.V_const_path; % constant translational velocity during straight movements
            stDim = MotionModel_class.stDim;
            ctDim=MotionModel_class.ctDim;
            r=MotionModel_class.robot_link_length;
            
<<<<<<< HEAD
            th_p = atan2( X_final(2)-X_initial(2)  ,  X_final(1)-X_initial(1)  ); % the angle of edge % note that "th_p" is already between -pi and pi, since it is the output of "atan2"
            %-------------- Rotation number of steps
            if abs(X_initial(3))>pi, X_initial(3)=(X_initial(3)-sign(X_initial(3))*2*pi); end % Here, we bound the initial angle "X_initial(3)" between -pi and pi
            if abs(X_final(3))>pi, X_final(3)=(X_final(3)-sign(X_final(3)*2*pi)); end % Here, we bound the final angle "X_final(3)" between -pi and pi
            delta_th_p = X_final(3) - X_initial(3); % turning angle
            if abs(delta_th_p)>pi, delta_th_p=(delta_th_p-sign(delta_th_p)*2*pi); end % Here, we bound "pre_delta_th_p" between -pi and pi
            rotation_steps = abs( delta_th_p/(omega_path*dt) );
            %--------------Translation number of steps
            delta_disp = norm( X_final(1:2) - X_initial(1:2) );
            translation_steps = abs(delta_disp/(V_path*dt));
            %--------------Total number of steps
            kf_rational = max([rotation_steps , translation_steps]);
            kf = floor(kf_rational)+1;  % note that in all following lines you cannot replace "floor(something)+1" by "ceil(something)", as it may be  a whole number.
            
            %=====================Rotation steps of the path
            delta_theta_const = omega_path*sign(delta_th_p)*dt;
            delta_theta_nominal(: , 1:floor(rotation_steps)) =  repmat( delta_theta_const , 1 , floor(rotation_steps));
            delta_theta_const_end = omega_path*sign(delta_th_p)*dt*(rotation_steps-floor(rotation_steps));
            delta_theta_nominal(:,floor(rotation_steps)+1) = delta_theta_const_end; % note that you cannot replace "floor(pre_rotation_steps)+1" by "ceil(pre_rotation_steps)", as it may be  a whole number.
            delta_theta_nominal = [delta_theta_nominal , zeros(1 , kf - size(delta_theta_nominal,2))]; % augment zeros to the end of "delta_theta_nominal", to make its length equal to "kf".
            
%             u_const = ones(3,1)*r*omega_path*sign(delta_th_p);
%             u_p_rot(: , 1:floor(rotation_steps)) = repmat( u_const,1,floor(rotation_steps) );
%             u_const_end = ones(3,1)*r*omega_path*sign(delta_th_p)*(rotation_steps-floor(rotation_steps));
%             u_p_rot(:,floor(rotation_steps)+1)=u_const_end; % note that you cannot replace "floor(pre_rotation_steps)+1" by "ceil(pre_rotation_steps)", as it may be  a whole number.
=======
            u_linear_ground  =  p *[u(1); 0; 0] ;
            w_linear_ground = p * [w(1); 0 ; 0];
            x_next_pos = pos + MotionModel_class.dt * (u_linear_ground) + ((MotionModel_class.dt^0.5) * w_linear_ground);

            u_angular_ground = [u(2); u(3); u(4)];%p * [u(2); u(3); u(4)];
            u_angular_ground = [u_angular_ground(1); u_angular_ground(2); u_angular_ground(3)];
            w_angular_ground = [w(2); w(3); w(4)]; % p * [w(2); w(3); w(4)]
            w_angular_ground = [w_angular_ground(1); w_angular_ground(2); w_angular_ground(3)];
%             q0 = x(4);
%             q1 = x(5);
%             q2 = x(6);
%             q3 = x(7);
            Amat = [-x(5),  -x(6), -x(7);
                      x(4)  , -x(7), x(6);
                     x(7)  , x(4),  -x(5);
                     -x(6),  x(5) ,  x(4)];
                 
            dq_dt_u = 0.5*Amat*u_angular_ground;
            dq_dt_w = 0.5*Amat*w_angular_ground;
             
            control = dq_dt_u * MotionModel_class.dt;
            noise = dq_dt_w * MotionModel_class.dt^0.5;
            x_next_rot = rot + control + noise;
            q_next = unit(x_next_rot); % Make a unit quaternion
            
            %             %transition_quat = MotionModel_class.f_transquat(MotionModel_class.dt , u_angular ,w_angular);
            %             x_next_q_rot = Quaternion();
            %             x_next_q_rot = unit(q_rot * transition_quat);% normalizing resultant quaternion)
            %             x_next_rot = double(x_next_q_rot);% converting to matrix f
          
            x_next = [x_next_pos(1)  x_next_pos(2)  x_next_pos(3) q_next(1) q_next(2) q_next(3) q_next(4)]'; % augmenting state and rotational part
        end

         function  J = df_dx_func(x,u,w) % state Jacobian
            pos = [x(1) , x(2) , x(3)];% position state
            rot  = [x(4) , x(5) , x(6) , x(7)];% rotation state
            q_rot = Quaternion(rot);
            q_rot = unit(q_rot);% making a normalized quarternion
            p = q_rot.R;
            u_angular = [u(2); u(3); u(4)];
            t3 = u_angular; % p * u_angular;
            u_angular_ground = [0 ;t3(1); t3(2); t3(3)];
            w_angular = [w(2); w(3); w(4)];
            t4 = w_angular; % p * w_angular;
            w_angular_ground = [0 ; t4(1); t4(2); t4(3)];
            a_11 = 1 ;
            a_12 = - 0.5 * t3(1) * MotionModel_class.dt - 0.5 * t4(1) * (MotionModel_class.dt)^0.5;
            a_13 = - 0.5 * t3(2) * MotionModel_class.dt - 0.5 * t4(2) * (MotionModel_class.dt)^0.5;
            a_14 = - 0.5 * t3(3) * MotionModel_class.dt - 0.5 * t4(3) * (MotionModel_class.dt)^0.5;
            a_21 =   0.5 * t3(1) * MotionModel_class.dt + 0.5 * t4(1) * (MotionModel_class.dt)^0.5;
            a_22 =  1 ;
            a_23 = 0.5 * t3(3) * MotionModel_class.dt + 0.5 * t4(3) * (MotionModel_class.dt)^0.5;
            a_24 = - 0.5 * t3(2) * MotionModel_class.dt - 0.5 * t4(2) * (MotionModel_class.dt)^0.5;
            a_31 = 0.5 * t3(2) * MotionModel_class.dt + 0.5 * t4(2) * (MotionModel_class.dt)^0.5;
            a_32 = - 0.5 * t3(3) * MotionModel_class.dt - 0.5 * t4(3) * (MotionModel_class.dt)^0.5;
            a_33 = 1 ;
            a_34 = 0.5 * t3(1) * MotionModel_class.dt + 0.5 * t4(1) * (MotionModel_class.dt)^0.5;
            a_41 = 0.5 * t3(3) * MotionModel_class.dt + 0.5 * t4(3) * (MotionModel_class.dt)^0.5;  
            a_42 = 0.5 * t3(2) * MotionModel_class.dt + 0.5 * t4(2) * (MotionModel_class.dt)^0.5;
            a_43 = -0.5 * t3(1) * MotionModel_class.dt - 0.5 * t4(1) * (MotionModel_class.dt)^0.5;
            a_44 = 1 ;
            A = [a_11 a_12 a_13 a_14; a_21 a_22 a_23 a_24; a_31 a_32 a_33 a_34; a_41 a_42 a_43 a_44];
            % Calculating the jacobian of the linear components B
            qq = q_rot.double; % put it back in double form
            q0 = qq(1);
            q1 = qq(2);
            q2 = qq(3);
            q3 = qq(4);
            Vsum = (u(1)*MotionModel_class.dt + w(1)*MotionModel_class.dt^0.5);
            b_11 = 1;
            b_12 = 0;
            b_13 = 0;
            b_14 = 2*q0 * Vsum;
            b_15 = 2*q1  * Vsum;
            b_16 = -2*q2 * Vsum;
            b_17 = -2*q3 * Vsum;
            b_21 = 0;
            b_22 = 1;
            b_23 = 0;
            b_24 = 2*q3*Vsum;
            b_25 = 2*q2*Vsum;
            b_26 = 2*q1*Vsum;
            b_27 = 2*q0*Vsum;
            b_31 = 0;
            b_32 = 0;
            b_33 = 1;
            b_34 = -2*q2*Vsum;
            b_35 = 2*q3*Vsum;
            b_36 = -2*q0*Vsum;
            b_37 = 2*q1*Vsum;
>>>>>>> ba6fc21e4458b0b1914888aaba4114631ed30a9f
            
            %=====================Translations
             delta_xy_const = [V_path*cos(th_p);V_path*sin(th_p)]*dt;
             delta_xy_nominal( : , 1:floor(translation_steps) ) = repmat( delta_xy_const , 1 , floor(translation_steps));
             delta_xy_const_end = [V_path*cos(th_p);V_path*sin(th_p)]*dt*(translation_steps - floor(translation_steps));
             delta_xy_nominal( : , floor(translation_steps)+1 ) = delta_xy_const_end;
            delta_xy_nominal = [delta_xy_nominal , zeros(2 , kf - size(delta_xy_nominal,2))]; % augment zeros to the end of "delta_xy_nominal", to make its length equal to "kf".
            
<<<<<<< HEAD
            
            delta_state_nominal = [delta_xy_nominal;delta_theta_nominal];
=======
            J = zeros(7,7);
            J(1:3,1:7) = B;
            J(4:7,4:7) = A; 
           
         end

        function J = df_du_func(x,u,w)
            rot  = [x(4) , x(5) , x(6) , x(7)];% rotation state
            q_rot = Quaternion(rot);
            q_rot = unit(q_rot);% making a normalized quarternion
            qq = q_rot.double; % put it back in double form
            R_gb = q_rot.R;
            q0 = qq(1);
            q1 = qq(2);
            q2 = qq(3);
            q3 = qq(4);
            
            b_11 = 0;
            b_12 = 0.5 *(-q1)* MotionModel_class.dt ;
            b_13 = 0.5 *(-q2)* MotionModel_class.dt ;
            b_14 = 0.5 *(-q3)* MotionModel_class.dt ;
            b_21 = 0 ;
            b_22 = 0.5 *(q0)* MotionModel_class.dt ;
            b_23 = 0.5 *(-q3)* MotionModel_class.dt ;
            b_24 = 0.5 *(q2)* MotionModel_class.dt ;
            b_31 = 0 ;
            b_32 = 0.5 *(q3)* MotionModel_class.dt ;
            b_33 = 0.5 *(q0)* MotionModel_class.dt ;
            b_34 = 0.5 *(-q1)* MotionModel_class.dt ;
            b_41 = 0;
            b_42 = 0.5 *(-q2)* MotionModel_class.dt ;
            b_43 = 0.5 *(q1) * MotionModel_class.dt ;
            b_44 = 0.5 *(q0) * MotionModel_class.dt ;
            B = [b_11 b_12 b_13 b_14; b_21 b_22 b_23 b_24; b_31 b_32 b_33 b_34; b_41 b_42 b_43 b_44];
            % Jacobian calc for linear part
            a_11 = (q0^2+q1^2-q2^2-q3^2) * MotionModel_class.dt ;
            a_12 = 0;
            a_13 = 0;
            a_14 = 0;
            a_21 = 2*(q1*q2+q0*q3)* MotionModel_class.dt;
            a_22 = 0;
            a_23 = 0;
            a_24 = 0;
            a_31 = 2*(q1*q3-q0*q2) * MotionModel_class.dt; 
            a_32 = 0;
            a_33 = 0;
            a_34 = 0;
            A = [a_11 a_12 a_13 a_14;a_21 a_22 a_23 a_24;a_31 a_32 a_33 a_34];
>>>>>>> ba6fc21e4458b0b1914888aaba4114631ed30a9f
            
            %=====================Nominal control and state trajectory generation
            x_p = zeros(stDim,kf+1);
            theta = zeros(1,kf+1);
            u_p = zeros(stDim,kf);
            
            x_p(:,1) = X_initial;
            theta(1) = X_initial(3);
            for k = 1:kf
                theta(:,k+1) = theta(:,k) + delta_state_nominal(3,k);
                th_k = theta(:,k);
                T_inv_k = [-sin(th_k),     cos(th_k)       ,r;
                 -sin(pi/3-th_k),-cos(pi/3-th_k) ,r;
                 sin(pi/3+th_k) ,-cos(pi/3+th_k) ,r];
             
                delta_body_velocities_k = delta_state_nominal(:,k)/dt; % x,y,and theta velocities in body coordinate at time step k
                u_p(:,k) = T_inv_k*delta_body_velocities_k;  % "T_inv_k" maps the "velocities in body coordinate" to the control signal
             
                x_p(:,k+1) = x_p(:,k) + delta_state_nominal(:,k);
            end
            
<<<<<<< HEAD
            % noiselss motion  % for debug: if you uncomment the following
            % lines you have to get the same "x_p_copy" as the "x_p"
            %             x_p_copy = zeros(stDim,kf+1);
            %             x_p_copy(:,1) = X_initial;
            %             for k = 1:kf
            %                 x_p_copy(:,k+1) = MotionModel_class.f_discrete(x_p_copy(:,k),u_p(:,k),zeros(MotionModel_class.wDim,1));
            %             end
=======
            g_11 = 0;
            g_12 = 0.5 *(-q1)* sqrt(MotionModel_class.dt) ;
            g_13 = 0.5 *(-q2)* sqrt(MotionModel_class.dt) ;
            g_14 = 0.5 *(-q3)* sqrt(MotionModel_class.dt) ;
            g_21 = 0 ;
            g_22 = 0.5 *(q0)* sqrt(MotionModel_class.dt);
            g_23 = 0.5 *(-q3)* sqrt(MotionModel_class.dt);
            g_24 = 0.5 *(q2)* sqrt(MotionModel_class.dt) ;
            g_31 = 0 ;
            g_32 = 0.5 *(q3)* sqrt(MotionModel_class.dt) ;
            g_33 = 0.5 *(q0)* sqrt(MotionModel_class.dt) ;
            g_34 = 0.5 *(-q1)* sqrt(MotionModel_class.dt) ;
            g_41 = 0;
            g_42 = 0.5 *(-q2)* sqrt(MotionModel_class.dt) ;
            g_43 = 0.5 *(q1)* sqrt(MotionModel_class.dt) ;
            g_44 = 0.5 *(q0)* sqrt(MotionModel_class.dt) ;
            G = [g_11 g_12 g_13 g_14; g_21 g_22 g_23 g_24; g_31 g_32 g_33 g_34; g_41 g_42 g_43 g_44];
            % Jacobian for linear part
            a_11 = (q0^2+q1^2-q2^2-q3^2) * (MotionModel_class.dt)^0.5 ;
            a_12 = 0;
            a_13 = 0;
            a_14 = 0;
            a_21 = 2*(q1*q2+q0*q3)* (MotionModel_class.dt)^0.5;
            a_22 = 0;
            a_23 = 0;
            a_24 = 0;
            a_31 = 2*(q1*q3-q0*q2) * (MotionModel_class.dt)^0.5; 
            a_32 = 0;
            a_33 = 0;
            a_34 = 0;
            A = [a_11 a_12 a_13 a_14;a_21 a_22 a_23 a_24;a_31 a_32 a_33 a_34];
            J = zeros(7,4);
            J(1:3,1:4) = A;
            J(4:7,1:4) = G;
>>>>>>> ba6fc21e4458b0b1914888aaba4114631ed30a9f
            
            nominal_traj.x = x_p;
            nominal_traj.u = u_p;
        end
<<<<<<< HEAD
        function nominal_traj = generate_VALID_open_loop_point2point_traj(X_initial,X_final) % generates open-loop trajectories between two start and goal states
            if isa(X_initial,'state'), X_initial=X_initial.val; end % retrieve the value of the state vector
            if isa(X_final,'state'), X_final=X_final.val; end % retrieve the value of the state vector
            % parameters
            omega_path=user_data_class.par.motion_model_parameters.omega_const_path; % constant rotational velocity during turnings
            dt=MotionModel_class.dt;
            V_path=user_data_class.par.motion_model_parameters.V_const_path; % constant translational velocity during straight movements
            stDim = MotionModel_class.stDim;
            ctDim=MotionModel_class.ctDim;
            r=MotionModel_class.robot_link_length;
=======
        function w = generate_process_noise(x,u) % simulate (generate) process noise based on the current poistion and controls
            [Un] =  MotionModel_class.generate_control_and_indep_process_noise(u);
            w = [Un];
        end
        function [Un] = generate_control_and_indep_process_noise(U)
            % generate Un
            indep_part_of_Un = randn(MotionModel_class.ctDim,1);
            P_Un =  MotionModel_class.control_noise_covariance(U);
            Un = indep_part_of_Un.*diag(P_Un.^(1/2));
        end
        function P_Un = control_noise_covariance(U)
            u_std=(MotionModel_class.eta_u).*U +(MotionModel_class.sigma_b_u);
            P_Un=diag(u_std.^2);
        end
        function Q_process_noise = process_noise_cov(x,u) % compute the covariance of process noise based on the current poistion and controls
            Q_process_noise = control_noise_covariance(u);
        end
        
        %THERE no validity checking, just implementing for tests sake
        function nominal_traj = generate_VALID_open_loop_point2point_traj(start,goal)
            nominal_traj = MotionModel_class.generate_open_loop_point2point_traj(start,goal);
        end
        
        function nominal_traj = generate_open_loop_point2point_traj(start,goal) % generates open-loop trajectories between two start and goal states
             % The MATLAB RVC ToolBox Must be loaded first
            disp('Using RRT3D to connect points on two orbits');
            disp('Start point is:');start
            disp('Goal point is:');goal
            disp('Solving...');
            veh = MotionModel_class();
            rrt = RRT3D([], veh, 'start', start, 'range', 5,'npoints',500,'speed',2,'time', MotionModel_class.dt);
            rrt.plan('goal',goal)   ;          % create navigation tree
            nominal_traj.x = [];
            nominal_traj.u = [];
            nominal_traj = rrt.path(start, goal) ; % animate path from this start location
>>>>>>> ba6fc21e4458b0b1914888aaba4114631ed30a9f
            
            th_p = atan2( X_final(2)-X_initial(2)  ,  X_final(1)-X_initial(1)  ); % the angle of edge % note that "th_p" is already between -pi and pi, since it is the output of "atan2"
            %-------------- Rotation number of steps
            if abs(X_initial(3))>pi, X_initial(3)=(X_initial(3)-sign(X_initial(3))*2*pi); end % Here, we bound the initial angle "X_initial(3)" between -pi and pi
            if abs(X_final(3))>pi, X_final(3)=(X_final(3)-sign(X_final(3)*2*pi)); end % Here, we bound the final angle "X_final(3)" between -pi and pi
            delta_th_p = X_final(3) - X_initial(3); % turning angle
            if abs(delta_th_p)>pi, delta_th_p=(delta_th_p-sign(delta_th_p)*2*pi); end % Here, we bound "pre_delta_th_p" between -pi and pi
            rotation_steps = abs( delta_th_p/(omega_path*dt) );
            %--------------Translation number of steps
            delta_disp = norm( X_final(1:2) - X_initial(1:2) );
            translation_steps = abs(delta_disp/(V_path*dt));
            %--------------Total number of steps
            kf_rational = max([rotation_steps , translation_steps]);
            kf = floor(kf_rational)+1;  % note that in all following lines you cannot replace "floor(something)+1" by "ceil(something)", as it may be  a whole number.
            
            %=====================Rotation steps of the path
            delta_theta_const = omega_path*sign(delta_th_p)*dt;
            delta_theta_nominal(: , 1:floor(rotation_steps)) =  repmat( delta_theta_const , 1 , floor(rotation_steps));
            delta_theta_const_end = omega_path*sign(delta_th_p)*dt*(rotation_steps-floor(rotation_steps));
            delta_theta_nominal(:,floor(rotation_steps)+1) = delta_theta_const_end; % note that you cannot replace "floor(pre_rotation_steps)+1" by "ceil(pre_rotation_steps)", as it may be  a whole number.
            delta_theta_nominal = [delta_theta_nominal , zeros(1 , kf - size(delta_theta_nominal,2))]; % augment zeros to the end of "delta_theta_nominal", to make its length equal to "kf".
            
%             u_const = ones(3,1)*r*omega_path*sign(delta_th_p);
%             u_p_rot(: , 1:floor(rotation_steps)) = repmat( u_const,1,floor(rotation_steps) );
%             u_const_end = ones(3,1)*r*omega_path*sign(delta_th_p)*(rotation_steps-floor(rotation_steps));
%             u_p_rot(:,floor(rotation_steps)+1)=u_const_end; % note that you cannot replace "floor(pre_rotation_steps)+1" by "ceil(pre_rotation_steps)", as it may be  a whole number.
            
            %=====================Translations
             delta_xy_const = [V_path*cos(th_p);V_path*sin(th_p)]*dt;
             delta_xy_nominal( : , 1:floor(translation_steps) ) = repmat( delta_xy_const , 1 , floor(translation_steps));
             delta_xy_const_end = [V_path*cos(th_p);V_path*sin(th_p)]*dt*(translation_steps - floor(translation_steps));
             delta_xy_nominal( : , floor(translation_steps)+1 ) = delta_xy_const_end;
            delta_xy_nominal = [delta_xy_nominal , zeros(2 , kf - size(delta_xy_nominal,2))]; % augment zeros to the end of "delta_xy_nominal", to make its length equal to "kf".
            
<<<<<<< HEAD
            
            delta_state_nominal = [delta_xy_nominal;delta_theta_nominal];
            
            %=====================Nominal control and state trajectory generation
            x_p = zeros(stDim,kf+1);
            theta = zeros(1,kf+1);
            u_p = zeros(stDim,kf);
            
            x_p(:,1) = X_initial;
            theta(1) = X_initial(3);
            for k = 1:kf
                theta(:,k+1) = theta(:,k) + delta_state_nominal(3,k);
                th_k = theta(:,k);
                T_inv_k = [-sin(th_k),     cos(th_k)       ,r;
                 -sin(pi/3-th_k),-cos(pi/3-th_k) ,r;
                 sin(pi/3+th_k) ,-cos(pi/3+th_k) ,r];
             
                delta_body_velocities_k = delta_state_nominal(:,k)/dt; % x,y,and theta velocities in body coordinate at time step k
                u_p(:,k) = T_inv_k*delta_body_velocities_k;  % "T_inv_k" maps the "velocities in body coordinate" to the control signal
             
                x_p(:,k+1) = x_p(:,k) + delta_state_nominal(:,k);
                %                 tmp.draw(); % FOR DEBUGGING
                tmp = state(x_p(:,k+1)); if tmp.is_constraint_violated, nominal_traj =[]; return; end
=======
            disp('Done with RRT...');
%             
%             x = start;
%             X = [];
%             X = [X,x];
%             w = [0,0,0,0]';
%             for i=1:length(controls(1,:))
%                 u = controls(:,i);
%                 x = MotionModel_class.f_discrete(x,u,w);
%                 X = [X,x];
%                 plot3(X(1,:),X(2,:),X(3,:),'x');
%                 pause(0.1);
%             end
        end
          %% Generate open-loop Orbit-to-Orbit trajectory
        function nominal_traj = generate_VALID_open_loop_orbit2orbit_traj(start_orbit, end_orbit) % generates open-loop trajectories between two start and end orbits
            % check if both the orbits are turning in the same
            % direction or not.
            direction_start_orbit = sign(start_orbit.u(1,1))*sign(start_orbit.u(2,1));
            direction_end_orbit = sign(end_orbit.u(1,1))*sign(end_orbit.u(2,1));
            % finding the connecting edge between orbits.
            if direction_start_orbit == direction_end_orbit % both orbits turn in a same direction
                gamma = atan2( end_orbit.center.val(2) - start_orbit.center.val(2) , end_orbit.center.val(1) - start_orbit.center.val(1) );
                temp_edge_start = start_orbit.radius * [ cos(gamma-pi/2) ; sin(gamma-pi/2);0 ] + start_orbit.center.val(1:3);
                temp_edge_end = end_orbit.radius* [ cos(gamma-pi/2) ; sin(gamma-pi/2);0 ] + end_orbit.center.val(1:3);
            else
                error('different directions have not been implemented in PNPRM yet.')
            end
            %temp_traj.x(:,1) = temp_edge_start;  temp_traj.x(:,2) = temp_edge_end;  % we generate this trajectory (only composed of start and end points) to check the collision probabilities before generating the edges.
            %collision = Unicycle_robot.is_constraints_violated(temp_traj);  % checking intersection with obstacles
            %             if collision == 1
            %                 nominal_traj = [];
            %                 return
            %             else
            %                 % construction edge trajectory
            roll_tmp_traj_start = 0;
            pitch_tmp_traj_start = 0;
            yaw_tmp_traj_start = gamma;
            q_tmp_traj_start = angle2quat(yaw_tmp_traj_start,pitch_tmp_traj_start,roll_tmp_traj_start);
    
            tmp_traj_start = [temp_edge_start ; q_tmp_traj_start' ];
            tmp_traj_goal = [temp_edge_end; q_tmp_traj_start'];
            nominal_traj = MotionModel_class.generate_open_loop_point2point_traj(tmp_traj_start,tmp_traj_goal);
        end
        %% Sample a valid orbit (periodic trajectory)
        function orbit = sample_a_valid_orbit()
            orbit_center = state.sample_a_valid_state();
            if isempty( orbit_center)
                orbit = [];
                return
            else
                orbit = MotionModel_class.generate_orbit(orbit_center);
                orbit = MotionModel_class.draw_orbit(orbit);
            end
            disp('fix the orbit drawing above in aircraft_kinematic.m Line 440')
        end
        %% Construct an orbit
        function orbit = generate_orbit(orbit_center)
            % minimum orbit radius resutls from dividing the minimum linear
            % velocity to maximum angular velocity. However, here we assume
            % that the linear velocity is constant.
            orbit.radius = MotionModel_class.turn_radius_min;
            orbit_length_meter = 2*pi*orbit.radius;
            orbit_length_time_continuous = orbit_length_meter/MotionModel_class.Min_Velocity;
            T_rational = orbit_length_time_continuous/MotionModel_class.dt;
            T = ceil(T_rational);
            orbit.period = T;
            orbit.center = orbit_center;
            
            % defining controls on the orbit
            V_p = MotionModel_class.Min_Velocity * [ones(1,T-1) , T_rational-floor(T_rational)]; % we traverse the orbit with minimum linear velocity
            omega_p = MotionModel_class.Max_Yaw_Rate * [ones(1,T-1) , T_rational-floor(T_rational)]; % we traverse the orbit with maximum angular velocity
            u_p = [V_p;zeros(1,T);zeros(1,T);omega_p];
            w_zero = MotionModel_class.zeroNoise; % no noise
>>>>>>> ba6fc21e4458b0b1914888aaba4114631ed30a9f
            
            end
            
            % noiselss motion  % for debug: if you uncomment the following
            % lines you have to get the same "x_p_copy" as the "x_p"
            %             x_p_copy = zeros(stDim,kf+1);
            %             x_p_copy(:,1) = X_initial;
            %             for k = 1:kf
            %                 x_p_copy(:,k+1) = MotionModel_class.f_discrete(x_p_copy(:,k),u_p(:,k),zeros(MotionModel_class.wDim,1));
            %             end
            
            nominal_traj.x = x_p;
            nominal_traj.u = u_p;
        end
        function YesNo = is_constraints_violated(open_loop_traj) % this function checks if the "open_loop_traj" violates any constraints or not. For example it checks collision with obstacles.
            % In this class the open loop trajectories are indeed straight
            % lines. So, we use following simplified procedure to check the
            % collisions.
            Obst=obstacles_class.obst;
            edge_start = open_loop_traj.x(1:2,1);
            edge_end = open_loop_traj.x(1:2,end);
            
            N_obst=size(Obst,2);
            intersection=0;
            for ib=1:N_obst
                X_obs=[Obst{ib}(:,1);Obst{ib}(1,1)];
                Y_obs=[Obst{ib}(:,2);Obst{ib}(1,2)];
                X_edge=[edge_start(1);edge_end(1)];
                Y_edge=[edge_start(2);edge_end(2)];
                [x_inters,~] = polyxpoly(X_obs,Y_obs,X_edge,Y_edge);
                if ~isempty(x_inters)
                    intersection=intersection+1;
                end
            end
            if intersection>0
                YesNo=1;
            else
                YesNo=0;
            end
        end
        function traj_plot_handle = draw_nominal_traj(nominal_traj, varargin)
            s_node_2D_loc = nominal_traj.x(1:2,1);
            e_node_2D_loc = nominal_traj.x(1:2,end);
            % retrieve PRM parameters provided by the user
            disp('the varargin need to be parsed here')
%             edge_spec = obj.par.edge_spec;
%             edge_width = obj.par.edge_width;
            edge_spec = '-b';
            edge_width = 2;

            % drawing the 2D edge line
            traj_plot_handle = plot([s_node_2D_loc(1),e_node_2D_loc(1)],[s_node_2D_loc(2),e_node_2D_loc(2)],edge_spec,'linewidth',edge_width);
        end
    end
    
    methods (Access = private)
        function nominal_traj = generate_open_loop_point2point_traj_turn_move_turn(obj , start_node_ind, end_node_ind)
            % I do not use this function anymore. But I kept it for future
            % references.
            X_initial = obj.nodes(start_node_ind).val;
            X_final = obj.nodes(end_node_ind).val;
            
            % parameters
            omega_path=user_data_class.par.motion_model_parameters.omega_const_path; % constant rotational velocity during turnings
            dt=user_data_class.par.motion_model_parameters.dt;
            V_path=user_data_class.par.motion_model_parameters.V_const_path; % constant translational velocity during straight movements
            stDim = MotionModel_class.stDim;
            ctDim=MotionModel_class.ctDim;
            r=MotionModel_class.robot_link_length;
            
            th_p = atan2( X_final(2)-X_initial(2)  ,  X_final(1)-X_initial(1)  ); % the angle of edge % note that "th_p" is already between -pi and pi, since it is the output of "atan2"
            %--------------Pre-Rotation number of steps
            if abs(X_initial(3))>pi, X_initial(3)=(X_initial(3)-sign(X_initial(3))*2*pi); end % Here, we bound the initial angle "X_initial(3)" between -pi and pi
            pre_delta_th_p = th_p - X_initial(3); % turning angle at the beginning of the edge (to align robot with edge)
            if abs(pre_delta_th_p)>pi, pre_delta_th_p=(pre_delta_th_p-sign(pre_delta_th_p)*2*pi); end % Here, we bound "pre_delta_th_p" between -pi and pi
            pre_rotation_steps = abs( pre_delta_th_p/(omega_path*dt) );
            %--------------Translation number of steps
            delta_disp = norm( X_final(1:2) - X_initial(1:2) );
            translation_steps = abs(delta_disp/(V_path*dt));
            %--------------Post-Rotation number of steps
            if abs(X_final(3))>pi, X_final(3)=(X_final(3)-sign(X_final(3)*2*pi)); end % Here, we bound the initial angle "X_final(3)" between -pi and pi
            post_delta_th_p =   X_final(3) - th_p; % turning angle at the end of the edge (to align robot with the end node)
            if abs(post_delta_th_p)>pi, post_delta_th_p=(post_delta_th_p-sign(post_delta_th_p)*2*pi); end % Here, we bound "post_delta_th_p" between -pi and pi
            post_rotation_steps = abs( post_delta_th_p/(omega_path*dt) );
            %--------------Total number of steps
            kf = floor(pre_rotation_steps)+1+floor(translation_steps)+1+floor(post_rotation_steps)+1;
            u_p=nan(ctDim,kf+1);
            
            %=====================Pre-Rotation
            u_const = ones(3,1)*r*omega_path*sign(pre_delta_th_p);
            u_p(: , 1:floor(pre_rotation_steps)) = repmat( u_const,1,floor(pre_rotation_steps) );
            u_const_end = ones(3,1)*r*omega_path*sign(pre_delta_th_p)*(pre_rotation_steps-floor(pre_rotation_steps));
            u_p(:,floor(pre_rotation_steps)+1)=u_const_end; % note that you cannot replace "floor(pre_rotation_steps)+1" by "ceil(pre_rotation_steps)", as it may be  a whole number.
            last_k = floor(pre_rotation_steps)+1;
            %=====================Translations
             T_inv = [-sin(th_p),     cos(th_p)       ,r;
                 -sin(pi/3-th_p),-cos(pi/3-th_p) ,r;
                 sin(pi/3+th_p) ,-cos(pi/3+th_p) ,r];
            u_const=T_inv*[V_path*cos(th_p);V_path*sin(th_p);0];
            u_p( : , last_k+1:last_k + floor(translation_steps) ) = repmat(u_const,1,floor(translation_steps));
            %Note that in below line we are using "u_const", in which the "Inv_Dyn"
            %has been already accounted for. So, we do not need to multiply it with
            %"Inv_Dyn" again.
            u_const_end = u_const*(translation_steps - floor(translation_steps));
            u_p( : , last_k + floor(translation_steps)+1 ) = u_const_end;
            %=====================Post-Rotation
            last_k = last_k + floor(translation_steps)+1;
            u_const = ones(3,1)*r*omega_path*sign(post_delta_th_p);
            u_p(: , last_k+1:last_k+floor(post_rotation_steps)) = repmat( u_const,1,floor(post_rotation_steps) );
            u_const_end = ones(3,1)*r*omega_path*sign(post_delta_th_p)*(post_rotation_steps-floor(post_rotation_steps));
            u_p(:,last_k+floor(post_rotation_steps)+1)=u_const_end; % note that you cannot replace "floor(pre_rotation_steps)+1" by "ceil(pre_rotation_steps)", as it may be  a whole number.
            
            % noiselss motion
            x_p = zeros(stDim,kf+1);
            x_p(:,1) = X_initial;
            for k = 1:kf
                x_p(:,k+1) = MotionModel_class.f_discrete(x_p(:,k),u_p(:,k),zeros(MotionModel_class.wDim,1));
            end
            
            nominal_traj.x = x_p;
            nominal_traj.u = u_p;
        end
    end
end


function [Un,Wg] = generate_control_and_indep_process_noise(U)
% generate Un
indep_part_of_Un = randn(MotionModel_class.ctDim,1);
P_Un = control_noise_covariance(U);
Un = indep_part_of_Un.*diag(P_Un.^(1/2));
% generate Wg
Wg = mvnrnd(zeros(MotionModel_class.stDim,1),MotionModel_class.P_Wg)';
end
function P_Un = control_noise_covariance(U)
u_std=(MotionModel_class.eta_u).*U+(MotionModel_class.sigma_b_u);
P_Un=diag(u_std.^2);
end