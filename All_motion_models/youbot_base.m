classdef youbot_base < MotionModel_interface
    % Note that because the class is defined as a handle class, the
    % properties must be defined such that they are do not change from an
    % object to another one.
    properties (Constant)
        stDim = state.dim; % state dimension
        ctDim = 4;  % control vector dimension
        wDim = 7;   % Process noise (W) dimension
        zeroControl = zeros(youbot_base.ctDim,1);
        zeroNoise = zeros(youbot_base.wDim,1);
        dt = user_data_class.par.motion_model_parameters.dt;
        l1 = 0.1585*2; % (meter) distance between right and left wheel 
        l2 =  0.228*2; % (meter) distance between front and rear wheel
        vMax = 0.8 % m/s maximum speed for individual wheels and max base velocity
        sigma_b_u = user_data_class.par.motion_model_parameters.sigma_b_u_KukaBase;
        eta_u = user_data_class.par.motion_model_parameters.eta_u_KukaBase;
        P_Wg = user_data_class.par.motion_model_parameters.P_Wg;
        
    end
    %     properties (Constant = true, SetAccess = private)
    %         UnDim = 3;
    %         WgDim = 3;
    %     end
    
    methods (Static)
        function x_next = f_discrete(x,u,w)
            Un = w(1:youbot_base.ctDim); % The size of Un may be different from ctDim in some other model.
            Wg = w(youbot_base.ctDim+1 : youbot_base.wDim); % The size of Wg may be different from stDim in some other model.
            Wc = youbot_base.f_contin(x,Un,Wg);
            x_next = x+youbot_base.f_contin(x,u,0)*youbot_base.dt+Wc*sqrt(youbot_base.dt);
%             x_next(3) = normAngle(x_next(3)); % Normalize angle to (-pi .. pi] values.
        end
        function x_dot = f_contin(x,u,wg) % Do not call this function from outside of this class!! % The last input in this method should be w, instead of wg. But, since it is a only used in this class, it does not matter so much.
            B = youbot_base.df_du_func(x,u,wg)/youbot_base.dt; % df_du_func is for the discrete model therefore it should be divided by dt to get the continuous B
            x_dot = B*u+wg;
        end
        function A = df_dx_func(x,u,w)
            A = eye(youbot_base.stDim);
        end
        function Acontin = df_contin_dx(x,u,w) %#ok<INUSD>
            Acontin = zeros(3,3);
        end
        function B = df_du_func(x,u,w) %#ok<INUSD>
            gama1= 2/(youbot_base.l2-youbot_base.l1);
            B     = (1/4)*[-1      1      1      -1;...
                           1       1      1      1;...
                           gama1 -gama1 gama1 -gama1]*youbot_base.dt; %% this is the B for the continous system 
            end
        function G = df_dw_func(x,u,w) %#ok<INUSD>
            B = youbot_base.df_du_func(x,u,w);
            G = [B/youbot_base.dt,eye(youbot_base.stDim)]*sqrt(youbot_base.dt); % this calculation is for the discrete system
        end
        function w = generate_process_noise(x,u) %#ok<INUSD>
            [Un,Wg] = generate_control_and_indep_process_noise(u);
            w = [Un;Wg];
        end
        function Q_process_noise = process_noise_cov(x,u) %#ok<INUSD>
            P_Un = control_noise_covariance(u);
            Q_process_noise = blkdiag(P_Un,youbot_base.P_Wg);
        end
        function nominal_traj = generate_open_loop_point2point_traj(X_initial,X_final) % generates open-loop trajectories between two start and goal states
            if isa(X_initial,'state'), X_initial=X_initial.val; end % retrieve the value of the state vector
            if isa(X_final,'state'), X_final=X_final.val; end % retrieve the value of the state vector
            B = youbot_base.df_du_func(0,0,0)/youbot_base.dt;; %% B matrix in the youbot robot does not depend on current state or input
            
            deltaX = (X_final - X_initial);
            
            t_interval = (1/0.8)*max(abs(B'*inv(B*B')*deltaX))/youbot_base.vMax; % we move by 80 percent of the maxmimum velocity
            kf = ceil(t_interval/ youbot_base.dt); % number of steps needed to follow the trajectory 
            uStar = (1/(kf*youbot_base.dt))*B'*inv(B*B')*deltaX;  %  
            
            %=====================Nominal control and state trajectory generation
            x_p = zeros(youbot_base.stDim,kf+1);
            u_p = zeros(youbot_base.ctDim,kf);
            
            x_p(:,1) = X_initial;
           
            for k = 1:kf
                u_p(:,k) = uStar;  % "T_inv_k" maps the "velocities in body coordinate" to the control signal
                x_p(:,k+1) = youbot_base.f_discrete(x_p(:,k),u_p(:,k) ,zeros(youbot_base.wDim,1)); % generaating the trajectory 

            end
            
            % noiselss motion  % for debug: if you uncomment the following
            % lines you have to get the same "x_p_copy" as the "x_p"
            %             x_p_copy = zeros(stDim,kf+1);
            %             x_p_copy(:,1) = X_initial;
            %             for k = 1:kf
            %                 x_p_copy(:,k+1) = youbot_base.f_discrete(x_p_copy(:,k),u_p(:,k),zeros(youbot_base.wDim,1));
            %             end
            
            nominal_traj.x = x_p;
            nominal_traj.u = u_p;
        end
        function nominal_traj = generate_VALID_open_loop_point2point_traj(X_initial,X_final) % generates open-loop trajectories between two start and goal states
            if isa(X_initial,'state'), X_initial=X_initial.val; end % retrieve the value of the state vector
            if isa(X_final,'state'), X_final=X_final.val; end % retrieve the value of the state vector
            B = youbot_base.df_du_func(0,0,0)/ youbot_base.dt;; %% B matrix in the youbot robot does not depend on current state or input
           
            deltaX = X_final - X_initial;
            
            t_interval = (1/0.8)*max(abs(B'*inv(B*B')*deltaX))/youbot_base.vMax; % we move by 80 percent of the maxmimum velocity
            kf = ceil(t_interval/ youbot_base.dt); % number of steps needed to follow the trajectory 
            uStar = (1/(kf*youbot_base.dt))*B'*inv(B*B')*deltaX;  %  
            
            %=====================Nominal control and state trajectory generation
            x_p = zeros(youbot_base.stDim,kf+1);
            u_p = zeros(youbot_base.ctDim,kf);
            
            x_p(:,1) = X_initial;
           
            for k = 1:kf
                u_p(:,k) = uStar;  % "T_inv_k" maps the "velocities in body coordinate" to the control signal
                x_p(:,k+1) = youbot_base.f_discrete(x_p(:,k),u_p(:,k) ,zeros(youbot_base.wDim,1)); % generaating the trajectory 
                tmp = state(x_p(:,k+1)); if tmp.is_constraint_violated(), nominal_traj =[]; return; end
            end
            
            % noiselss motion  % for debug: if you uncomment the following
            % lines you have to get the same "x_p_copy" as the "x_p"
            %             x_p_copy = zeros(stDim,kf+1);
            %             x_p_copy(:,1) = X_initial;
            %             for k = 1:kf
            %                 x_p_copy(:,k+1) = youbot_base.f_discrete(x_p_copy(:,k),u_p(:,k),zeros(youbot_base.wDim,1));
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
            stDim = youbot_base.stDim;
            ctDim=youbot_base.ctDim;
            r=youbot_base.robot_link_length;
            
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
                x_p(:,k+1) = youbot_base.f_discrete(x_p(:,k),u_p(:,k),zeros(youbot_base.wDim,1));
            end
            
            nominal_traj.x = x_p;
            nominal_traj.u = u_p;
        end
    end
end


function [Un,Wg] = generate_control_and_indep_process_noise(U)
% generate Un
indep_part_of_Un = randn(youbot_base.ctDim,1);
P_Un = control_noise_covariance(U);
Un = indep_part_of_Un.*diag(P_Un.^(1/2));
% generate Wg
Wg = mvnrnd(zeros(youbot_base.stDim,1),youbot_base.P_Wg)';
end
function P_Un = control_noise_covariance(U)
u_std=(youbot_base.eta_u).*U+(youbot_base.sigma_b_u);
P_Un=diag(u_std.^2);
end