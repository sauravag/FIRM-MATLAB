classdef MotionModel_class < MotionModel_interface
    % Note that because the class is defined as a handle class, the
    % properties must be defined such that they are do not change from an
    % object to another one.
    properties (Constant)
        num_robots = user_data_class.par.state_parameters.num_robots;
        stDim = state.dim; % state dimension
        ctDim = state.dim;  % control vector dimension
        wDim = 2*state.dim;   % Process noise (W) dimension
        dt = user_data_class.par.motion_model_parameters.dt;
        robot_link_length = user_data_class.par.motion_model_parameters.robot_link_length;
        sigma_b_u = user_data_class.par.motion_model_parameters.sigma_b_u_omni_team;
        eta_u = user_data_class.par.motion_model_parameters.eta_u_omni_team;
        P_Wg = user_data_class.par.motion_model_parameters.P_Wg_team;
    end
    
    methods (Static)
        function x_next_team = f_discrete(x_team,u_team,w_team)
            for i = 1:MotionModel_class.num_robots
                x = x_team(3*i-2:3*i);
                u = u_team(3*i-2:3*i);
                Un = w_team(6*i-5:6*i-3);
                Wg = w_team(6*i-2:6*i);
                Wc = MotionModel_class.f_contin(x,Un,Wg);
                x_next = x+MotionModel_class.f_contin(x,u,0)*MotionModel_class.dt+Wc*sqrt(MotionModel_class.dt);
                x_next_team(3*i-2:3*i , 1) = x_next; % Note that the "x_next_team" must be a column vector
            end
        end
        function x_dot = f_contin(x,u,wg) % Do not call this function from outside of this class!! % The last input in this method should be w, instead of wg. But, since it is a only used in this class, it does not matter so much.
            % Note that this function should only work for a single robot,
            % Not for entire team.
            th = x(3);
            r = MotionModel_class.robot_link_length;
            x_dot = (1/3)*[-2*sin(th),-2*sin(pi/3-th),2*sin(pi/3+th);
                2*cos(th),-2*cos(pi/3-th),-2*cos(pi/3+th);
                1/r,1/r,1/r]*u+wg;
        end
        function A_team = df_dx_func(x_team,u_team,w_team)
            A_team = [];
            for i = 1:MotionModel_class.num_robots
                x = x_team(3*i-2:3*i);
                u = u_team(3*i-2:3*i);
                un = w_team(6*i-5:6*i-3);
                wg = w_team(6*i-2:6*i);
                A = eye(3) ...
                    + MotionModel_class.df_contin_dx(x,u,zeros(3,1))*MotionModel_class.dt ...
                    + MotionModel_class.df_contin_dx(x,un,wg)*sqrt(MotionModel_class.dt);
                A_team = blkdiag(A_team,A);
            end
        end
        function Acontin = df_contin_dx(x,u,w) %#ok<INUSD>
            % Note that this function should only work for a single robot,
            % Not for entire team.
            th = x(3);
            Acontin = (2/3)*[0 0 -cos(th)*u(1)+cos(pi/3-th)*u(2)+cos(pi/3+th)*u(3);
                0 0 -sin(th)*u(1)-sin(pi/3-th)*u(2)+sin(pi/3+th)*u(3);
                0 0 0];
        end
        function B_team = df_du_func(x_team,u_team,w_team) %#ok<INUSD>
            B_team = [];
            for i = 1:MotionModel_class.num_robots
                th = x_team(3*i);
                r = MotionModel_class.robot_link_length;
                B = (1/3)*[-2*sin(th),-2*sin(pi/3-th),2*sin(pi/3+th);
                    2*cos(th) ,-2*cos(pi/3-th),-2*cos(pi/3+th);
                    1/r       ,1/r            ,1/r            ]*MotionModel_class.dt;
                B_team = blkdiag(B_team,B);
            end
        end
        function G_team = df_dw_func(x_team,u_team,w_team) %#ok<INUSD>
            G_team = [];
            for i = 1:MotionModel_class.num_robots
                th=x_team(3*i);
                r=MotionModel_class.robot_link_length;
                T_theta=(1/3)*[-2*sin(th),-2*sin(pi/3-th),2*sin(pi/3+th);
                    2*cos(th) ,-2*cos(pi/3-th),-2*cos(pi/3+th);
                    1/r       ,1/r            ,1/r            ];
                G = [T_theta,eye(3)]*sqrt(MotionModel_class.dt);
                G_team = blkdiag(G_team,G);
            end
        end
        function w_team = generate_process_noise(x_team,u_team) %#ok<INUSD>
            [Un_team,Wg_team] = generate_control_and_indep_process_noise(u_team);
            w_team = [];
            for i = 1:MotionModel_class.num_robots
                w_team = [w_team ; Un_team(3*i-2:3*i);Wg_team(3*i-2:3*i)]; %#ok<AGROW> % in the full noise vector, for each robot we first store the Un and then Wg.
            end
        end
        function Q_process_noise_team = process_noise_cov(x_team,u_team) %#ok<INUSD>
            P_Un_team = control_noise_covariance(u_team);
            Q_process_noise_team = [];
            for i = 1:MotionModel_class.num_robots
                Q_process_noise_team = blkdiag(Q_process_noise_team , P_Un_team(3*i-2:3*i , 3*i-2:3*i),MotionModel_class.P_Wg(3*i-2:3*i , 3*i-2:3*i)); % in the full noise matrix, for each robot we first store the Un cov and then Wg cov.
            end
        end
        function nominal_traj_team = generate_open_loop_point2point_traj(X_initial_team,X_final_team) % generates open-loop trajectories between two start and goal states
            if isa(X_initial_team,'state'), X_initial_team=X_initial_team.val; end % retrieve the value of the state vector
            if isa(X_final_team,'state'), X_final_team=X_final_team.val; end % retrieve the value of the state vector
            % parameters (same for all team)
            dt=MotionModel_class.dt;
            stDim_team = MotionModel_class.stDim;
            ctDim_team = MotionModel_class.ctDim;
            r=MotionModel_class.robot_link_length;
            for i = 1:MotionModel_class.num_robots
                % parameters (possibly different for each robot)
                stDim = stDim_team/MotionModel_class.num_robots;
                ctDim = ctDim_team/MotionModel_class.num_robots;
                omega_path=user_data_class.par.motion_model_parameters.omega_const_path_team(i); % constant rotational velocity during turnings
                V_path=user_data_class.par.motion_model_parameters.V_const_path_team(i); % constant translational velocity during straight movements
                
                X_initial = X_initial_team(3*i-2:3*i);
                X_final = X_final_team(3*i-2:3*i);
                
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
                end
                
                % noiselss motion  % for debug: if you uncomment the following
                % lines you have to get the same "x_p_copy" as the "x_p"
                %             x_p_copy = zeros(stDim,kf+1);
                %             x_p_copy(:,1) = X_initial;
                %             for k = 1:kf
                %                 x_p_copy(:,k+1) = MotionModel_class.f_discrete(x_p_copy(:,k),u_p(:,k),zeros(MotionModel_class.wDim,1));
                %             end
                
                x_p_team{i} = x_p; %#ok<AGROW>
                u_p_team{i} = u_p; %#ok<AGROW>
                kf_team(i) = kf; %#ok<AGROW>
            end
            max_kf = max(kf_team);
            nominal_traj_team.x = [];nominal_traj_team.u = [];
            for i = 1:MotionModel_class.num_robots
                nominal_traj_team.x = [nominal_traj_team.x ; [x_p_team{i}  ,  repmat( x_p_team{i}(:,end) , 1 , max_kf+1-size(x_p_team{i},2))] ];
                nominal_traj_team.u = [nominal_traj_team.u ; [u_p_team{i} ,  zeros(3,max_kf-size(u_p_team{i},2))] ];
            end
        end
        function YesNo = is_constraints_violated(open_loop_traj) % this function checks if the "open_loop_traj" violates any constraints or not. For example it checks collision with obstacles.
            % In this class the open loop trajectories are indeed straight
            % lines. So, we use following simplified procedure to check the
            % collisions.
            YesNo=0; % initialization
            Obst=obstacles_class.obst;
            for j = 1:MotionModel_class.num_robots
                edge_start = open_loop_traj.x(3*j-2:3*j-1  , 1);
                edge_end = open_loop_traj.x(3*j-2:3*j-1   , end);
                
                N_obst=size(Obst,2);
                for ib=1:N_obst
                    X_obs=[Obst{ib}(:,1);Obst{ib}(1,1)];
                    Y_obs=[Obst{ib}(:,2);Obst{ib}(1,2)];
                    X_edge=[edge_start(1);edge_end(1)];
                    Y_edge=[edge_start(2);edge_end(2)];
                    [x_inters,~] = polyxpoly(X_obs,Y_obs,X_edge,Y_edge);
                    if ~isempty(x_inters)
                        YesNo=1;
                        return
                    end
                end
            end
        end
        function traj_plot_handle = draw_nominal_traj(nominal_traj)
            traj_plot_handle = [];
            for i = 1:MotionModel_class.num_robots
            s_node_2D_loc = nominal_traj.x( 3*i-2:3*i-1  ,  1);
            e_node_2D_loc = nominal_traj.x( 3*i-2:3*i-1  ,  end);
            % retrieve PRM parameters provided by the user
            disp('the varargin need to be parsed here')
%             edge_spec = obj.par.edge_spec;
%             edge_width = obj.par.edge_width;
            edge_spec = '-b';
            edge_width = 2;

            % drawing the 2D edge line
            tmp_h = plot([s_node_2D_loc(1),e_node_2D_loc(1)],[s_node_2D_loc(2),e_node_2D_loc(2)],edge_spec,'linewidth',edge_width);
            traj_plot_handle = [traj_plot_handle , tmp_h]; %#ok<AGROW>
            end
        end
    end
end


function [Un_team,Wg_team] = generate_control_and_indep_process_noise(U_team)
% generate Un
indep_part_of_Un_team = randn(MotionModel_class.ctDim,1);
P_Un_team = control_noise_covariance(U_team);
Un_team = indep_part_of_Un_team.*diag(P_Un_team.^(1/2));
% generate Wg
Wg_team = mvnrnd(zeros(MotionModel_class.stDim,1),MotionModel_class.P_Wg)';
end
function P_Un_team = control_noise_covariance(U_team)
u_std_team=(MotionModel_class.eta_u).*U_team+(MotionModel_class.sigma_b_u); % note that "MotionModel_class.eta_u" has to be the same size as the "U_team"
P_Un_team=diag(u_std_team.^2);
end