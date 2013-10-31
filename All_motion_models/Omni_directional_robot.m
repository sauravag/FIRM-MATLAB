classdef Omni_directional_robot < MotionModel_interface
    % Note that because the class is defined as a handle class, the
    % properties must be defined such that they are do not change from an
    % object to another one.
    properties (Constant)
        stDim = state.dim; % state dimension
        ctDim = 3;  % control vector dimension
        wDim = 6;   % Process noise (W) dimension
        zeroControl = zeros(Omni_directional_robot.ctDim,1);
        zeroNoise = zeros(Omni_directional_robot.wDim,1);
        dt = user_data_class.par.motion_model_parameters.dt;
        robot_link_length = user_data_class.par.motion_model_parameters.robot_link_length;
        sigma_b_u = user_data_class.par.motion_model_parameters.sigma_b_u_omni;
        eta_u = user_data_class.par.motion_model_parameters.eta_u_omni;
        P_Wg = user_data_class.par.motion_model_parameters.P_Wg;
    end
    %     properties (Constant = true, SetAccess = private)
    %         UnDim = 3;
    %         WgDim = 3;
    %     end
    
    methods (Static)
        function x_next = f_discrete(x,u,w)
            Un = w(1:Omni_directional_robot.ctDim); % The size of Un may be different from ctDim in some other model.
            Wg = w(Omni_directional_robot.ctDim+1 : Omni_directional_robot.wDim); % The size of Wg may be different from stDim in some other model.
            Wc = Omni_directional_robot.f_contin(x,Un,Wg);
            x_next = x+Omni_directional_robot.f_contin(x,u,0)*Omni_directional_robot.dt+Wc*sqrt(Omni_directional_robot.dt);
        end
        function x_dot = f_contin(x,u,wg) % Do not call this function from outside of this class!! % The last input in this method should be w, instead of wg. But, since it is a only used in this class, it does not matter so much.
            th = x(3);
            r = Omni_directional_robot.robot_link_length;
            x_dot = (1/3)*[-2*sin(th),-2*sin(pi/3-th),2*sin(pi/3+th);
                2*cos(th),-2*cos(pi/3-th),-2*cos(pi/3+th);
                1/r,1/r,1/r]*u+wg;
        end
        function A = df_dx_func(x,u,w)
            un = w(1:Omni_directional_robot.ctDim); % The size of Un may be different from ctDim in some other model.
            wg = w(Omni_directional_robot.ctDim+1 : Omni_directional_robot.wDim); % The size of Wg may be different from stDim in some other model.
            A = eye(Omni_directional_robot.stDim) ...
                + Omni_directional_robot.df_contin_dx(x,u,zeros(Omni_directional_robot.stDim,1))*Omni_directional_robot.dt ...
                + Omni_directional_robot.df_contin_dx(x,un,wg)*sqrt(Omni_directional_robot.dt);
        end
        function Acontin = df_contin_dx(x,u,w) %#ok<INUSD>
            th = x(3);
            Acontin = (2/3)*[0 0 -cos(th)*u(1)+cos(pi/3-th)*u(2)+cos(pi/3+th)*u(3);
                0 0 -sin(th)*u(1)-sin(pi/3-th)*u(2)+sin(pi/3+th)*u(3);
                0 0 0];
        end
        function B = df_du_func(x,u,w) %#ok<INUSD>
            th = x(3);
            r = Omni_directional_robot.robot_link_length;
            B = (1/3)*[-2*sin(th),-2*sin(pi/3-th),2*sin(pi/3+th);
                2*cos(th) ,-2*cos(pi/3-th),-2*cos(pi/3+th);
                1/r       ,1/r            ,1/r            ]*Omni_directional_robot.dt;
        end
        function G = df_dw_func(x,u,w) %#ok<INUSD>
            th=x(3);
            r=Omni_directional_robot.robot_link_length;
            T_theta=(1/3)*[-2*sin(th),-2*sin(pi/3-th),2*sin(pi/3+th);
                2*cos(th) ,-2*cos(pi/3-th),-2*cos(pi/3+th);
                1/r       ,1/r            ,1/r            ];
            G = [T_theta,eye(Omni_directional_robot.stDim)]*sqrt(Omni_directional_robot.dt);
        end
        function w = generate_process_noise(x,u) %#ok<INUSD>
            [Un,Wg] = generate_control_and_indep_process_noise(u);
            w = [Un;Wg];
        end
        function Q_process_noise = process_noise_cov(x,u) %#ok<INUSD>
            P_Un = control_noise_covariance(u);
            Q_process_noise = blkdiag(P_Un,Omni_directional_robot.P_Wg);
        end
        function nominal_traj = generate_VALID_open_loop_point2point_traj(X_initial,X_final) % generates open-loop trajectories between two start and goal states
            if isa(X_initial,'state'), X_initial=X_initial.val; end % retrieve the value of the state vector
            if isa(X_final,'state'), X_final=X_final.val; end % retrieve the value of the state vector
            % parameters
            omega_path=user_data_class.par.motion_model_parameters.omega_const_path; % constant rotational velocity during turnings
            dt=Omni_directional_robot.dt;
            V_path=user_data_class.par.motion_model_parameters.V_const_path; % constant translational velocity during straight movements
            stDim = Omni_directional_robot.stDim;
            ctDim=Omni_directional_robot.ctDim;
            r=Omni_directional_robot.robot_link_length;
            
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
                tmp = state(x_p(:,k+1)); if tmp.is_constraint_violated, nominal_traj =[]; return; end
            end
            
            % noiselss motion  % for debug: if you uncomment the following
            % lines you have to get the same "x_p_copy" as the "x_p"
            %             x_p_copy = zeros(stDim,kf+1);
            %             x_p_copy(:,1) = X_initial;
            %             for k = 1:kf
            %                 x_p_copy(:,k+1) = Omni_directional_robot.f_discrete(x_p_copy(:,k),u_p(:,k),zeros(Omni_directional_robot.wDim,1));
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
            stDim = Omni_directional_robot.stDim;
            ctDim=Omni_directional_robot.ctDim;
            r=Omni_directional_robot.robot_link_length;
            
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
                x_p(:,k+1) = Omni_directional_robot.f_discrete(x_p(:,k),u_p(:,k),zeros(Omni_directional_robot.wDim,1));
            end
            
            nominal_traj.x = x_p;
            nominal_traj.u = u_p;
        end
    end
end


function [Un,Wg] = generate_control_and_indep_process_noise(U)
% generate Un
indep_part_of_Un = randn(Omni_directional_robot.ctDim,1);
P_Un = control_noise_covariance(U);
Un = indep_part_of_Un.*diag(P_Un.^(1/2));
% generate Wg
Wg = mvnrnd(zeros(Omni_directional_robot.stDim,1),Omni_directional_robot.P_Wg)';
end
function P_Un = control_noise_covariance(U)
u_std=(Omni_directional_robot.eta_u).*U+(Omni_directional_robot.sigma_b_u);
P_Un=diag(u_std.^2);
end