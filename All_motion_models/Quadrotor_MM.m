classdef Quadrotor_MM < MotionModel_interface
    % Note that because the class is defined as a handle class, the
    % properties must be defined such that they are do not change from an
    % object to another one.
    properties (Constant)
        stDim = state.dim; % state dimension
        ctDim = 4;  % control vector dimension
        wDim = 16;   % Process noise (W) dimension - 4 for control noise and 12 for additive state noise
        zeroControl = zeros(Quadrotor_MM.ctDim,1);
        zeroNoise = zeros(Quadrotor_MM.wDim,1);
        dt = user_data_class.par.motion_model_parameters.dt;
        g = 9.81; % (m/s^2)  --  gravitational constant 
        mass =  0.650; % (kg) quadrotor mass
        len = 0.23 % (m) length of the quadrotor
        EyeX = 0.0075; % (kg*m2) moments of intertia -- x coordinate
        EyeY = 0.0075; % (kg*m2) moments of intertia -- y coordinate
        EyeZ = 0.013; % (kg*m2) moments of intertia -- z coordinate
        sigma_b_u = user_data_class.par.motion_model_parameters.sigma_b_u_quadrotor;
        eta_u = user_data_class.par.motion_model_parameters.eta_u_quadrotor;
        P_Wg = user_data_class.par.motion_model_parameters.P_Wg;
    end
    
    methods (Static)
        function x_next = f_discrete(x,u,w)
            x_next = Quadrotor_MM.df_dx_func(x,u,w)*x+Quadrotor_MM.df_du_func(x,u,w)*u+Quadrotor_MM.df_dw_func(x,u,w)*w;
        end
        function A = df_dx_func(x,u,w) %#ok<INUSD>
            Acontin = zeros(12,12);
            Acontin(1:9,4:12)=diag([1,1,1,Quadrotor_MM.g,-Quadrotor_MM.g,0,1,1,1]);
            A = eye(12) + Acontin*Quadrotor_MM.dt;
        end
        function Bcontin = df_contin_du_func(x,u,w) %#ok<INUSD>
            Bcontin = zeros(12,4);
            Bcontin(6,1) = 1/Quadrotor_MM.mass;
            Bcontin(9:12, 1:4) = diag([0, Quadrotor_MM.len/Quadrotor_MM.EyeX, Quadrotor_MM.len/Quadrotor_MM.EyeY, 1/Quadrotor_MM.EyeZ]); 
        end
        function B = df_du_func(x,u,w)
            B = Quadrotor_MM.df_contin_du_func(x,u,w)*Quadrotor_MM.dt;
        end
        function G = df_dw_func(x,u,w)
            Gcontin = [Quadrotor_MM.df_contin_du_func(x,u,w),eye(12)];
            G = Gcontin*sqrt(Quadrotor_MM.dt);
        end
        function w = generate_process_noise(x,u) %#ok<INUSD>
            [Un,Wg] = generate_control_and_indep_process_noise(u);
            w = [Un;Wg];
        end
        function Q_process_noise = process_noise_cov(x,u) %#ok<INUSD>
            P_Un = control_noise_covariance(u);
            Q_process_noise = blkdiag(P_Un,Quadrotor_MM.P_Wg);
        end
        function nominal_traj = generate_open_loop_point2point_traj(X_initial,X_final) % generates open-loop trajectories between two start and goal states
            if isa(X_initial,'state'), X_initial=X_initial.val; end % retrieve the value of the state vector
            if isa(X_final,'state'), X_final=X_final.val; end % retrieve the value of the state vector
            
            params.g = Quadrotor_MM.g;
            params.m = Quadrotor_MM.mass; % quadrotor mass kg
            params.Ix = Quadrotor_MM.EyeX; % Moment of Inertia kg*m2
            params.Iy = Quadrotor_MM.EyeY; % Moment of Inertia kg*m2
            params.Iz = Quadrotor_MM.EyeZ; % Moment of Inertia kg*m2
            params.L = Quadrotor_MM.len; % Arm Length m
            params.dt = Quadrotor_MM.dt;
            
            %% Initial and Final States
            
            %%%%%%%%%%
            % IN THIS FILE THE ORDER OF P,ATT,PDOT,ATTDOT has been changed
            % to P, PDOT, ATT, ATTDOT (just locally in this function -- not in
            % general) -- So, the X_initial and x_init are different.
            
            % Initial State 
            p_init = X_initial(1:3,1);
            att_init = X_initial(4:6,1);
            pdot_init = X_initial(7:9,1);
            attdot_init = X_initial(10:12,1);
            
            x_init = [p_init;pdot_init;att_init;attdot_init]; % NOT THE SAME AS X_initial
            
            % Final State
            p_final = X_final(1:3,1);
             att_final = X_final(4:6,1);
            pdot_final = X_final(7:9,1);
            attdot_final = X_final(10:12,1);
            
            x_final = [p_final;pdot_final;att_final;attdot_final]; % NOT THE SAME AS X_final
            
            n_steps = 35;
            
            % Reference States (X,Xdot and Y,Ydot)
            
            % Fit a second order polynomial path
            
            delta_x = p_final(1) -p_init(1);
            
            time = 0:params.dt:(n_steps-1)*params.dt;
            
            refs = zeros(8,n_steps);
            
            refs(1,:) = p_final(1); % px command
            refs(5,:) = p_final(2); % py command
            
          
            
            %% Propogate with FL Control laws
            
            states = zeros(12,n_steps);
            
            states(:,1) = x_init;
            
            u = zeros(4,n_steps-1);
            
            for time_index=2:n_steps
                
                % Control
                
                u(:,time_index-1) =  QuadFeedbackLinearization(states(:,time_index-1),...
                    refs(:,time_index-1),params);
                
                
                % Integration
                states(:,time_index) =...
                    quadDyn(states(:,time_index-1),u(:,time_index-1),params)*params.dt +...
                    states(:,time_index-1);
                
            end
            
            xp(1:3,:) = states(1:3,:);
            xp(4:6,:) = states(7:9,:);
            xp(7:9,:) = states(4:6,:);
            xp(10:12,:) = states(10:12,:);
            
            nominal_traj.x = xp;
            nominal_traj.u = u;
        end
        function nominal_traj = generate_VALID_open_loop_point2point_traj(X_initial,X_final) % generates open-loop trajectories between two start and goal states
            if isa(X_initial,'state'), X_initial=X_initial.val; end % retrieve the value of the state vector
            if isa(X_final,'state'), X_final=X_final.val; end % retrieve the value of the state vector
            
            params.g = Quadrotor_MM.g;
            params.m = Quadrotor_MM.mass; % quadrotor mass kg
            params.Ix = Quadrotor_MM.EyeX; % Moment of Inertia kg*m2
            params.Iy = Quadrotor_MM.EyeY; % Moment of Inertia kg*m2
            params.Iz = Quadrotor_MM.EyeZ; % Moment of Inertia kg*m2
            params.L = Quadrotor_MM.len; % Arm Length m
            params.dt = Quadrotor_MM.dt;
            
            %% Initial and Final States
            
            %%%%%%%%%%
            % IN THIS FILE THE ORDER OF P,ATT,PDOT,ATTDOT has been changed
            % to P, PDOT, ATT, ATTDOT (just locally in this function -- not in
            % general) -- So, the X_initial and x_init are different.
            
            % Initial State 
            p_init = X_initial(1:3,1);
            att_init = X_initial(4:6,1);
            pdot_init = X_initial(7:9,1);
            attdot_init = X_initial(10:12,1);
            
            x_init = [p_init;pdot_init;att_init;attdot_init]; % NOT THE SAME AS X_initial
            
            % Final State
            p_final = X_final(1:3,1);
             att_final = X_final(4:6,1);
            pdot_final = X_final(7:9,1);
            attdot_final = X_final(10:12,1);
            
            x_final = [p_final;pdot_final;att_final;attdot_final]; % NOT THE SAME AS X_final
            
            n_steps = 35;
            
            % Reference States (X,Xdot and Y,Ydot)
            
            % Fit a second order polynomial path
            
            delta_x = p_final(1) -p_init(1);
            
            time = 0:params.dt:(n_steps-1)*params.dt;
            
            refs = zeros(10,n_steps);
            
            refs(1,:) = p_final(1); % px command
            refs(5,:) = p_final(2); % py command
            refs(9,:) = p_final(3); % pz command
          
            
            %% Propogate with FL Control laws
            
            states = zeros(12,n_steps);
            
            states(:,1) = x_init;
            
            u = zeros(4,n_steps-1);
            
            for time_index=2:n_steps
                
                % Control
                
                u(:,time_index-1) =  QuadFeedbackLinearization(states(:,time_index-1),...
                    refs(:,time_index-1),params);
                
                
                % Integration
                states(:,time_index) =...
                    quadDyn(states(:,time_index-1),u(:,time_index-1),params)*params.dt +...
                    states(:,time_index-1);
                
            end
            
            xp(1:3,:) = states(1:3,:);
            xp(4:6,:) = states(7:9,:);
            xp(7:9,:) = states(4:6,:);
            xp(10:12,:) = states(10:12,:);
            
            nominal_traj.x = xp;
            nominal_traj.u = u;

        end
        function YesNo = is_constraints_violated(open_loop_traj) % this function checks if the "open_loop_traj" violates any constraints or not. For example it checks collision with obstacles.
            error('not implemented yet')
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
            for k = 1:length(nominal_traj.x)
                xTmp = state(nominal_traj.x(:,k));
                xTmp = xTmp.draw('robotshape','triangle');
            end
            traj_plot_handle = [];
%             s_node_2D_loc = nominal_traj.x(1:2,1);
%             e_node_2D_loc = nominal_traj.x(1:2,end);
%             % retrieve PRM parameters provided by the user
%             disp('the varargin need to be parsed here')
% %             edge_spec = obj.par.edge_spec;
% %             edge_width = obj.par.edge_width;
%             edge_spec = '-b';
%             edge_width = 2;
% 
%             % drawing the 2D edge line
%             traj_plot_handle = plot([s_node_2D_loc(1),e_node_2D_loc(1)],[s_node_2D_loc(2),e_node_2D_loc(2)],edge_spec,'linewidth',edge_width);
        end
    end
    
    methods (Static, Access = private)
        function nominal_traj = generate_open_loop_point2point_traj_turn_move_turn(obj , start_node_ind, end_node_ind)
            error('not implemented yet')
        end
        function d_yaw_seq = steerHeading(psi_init,psi_final,n_steps)
            % Heading Control sub-system (states: psi and r)
            A = [0,1;0 0];
            B = [0;1/(Quadrotor_MM.EyeZ)];
            lower = [-pi/2;-pi;-1];
            upper = [pi/2;pi;1];
            x_init = [psi_init,0];
            x_final = [psi_final,0];
            [~,d_yaw_seq] = steerLinearSystembyLP(A,B,lower,upper,x_init,x_final,Quadrotor_MM.dt,n_steps);
        end
        
        function  d_roll_seq = steerX(px_init,px_final,n_steps)
            % X Control sub-system(states x xdot phi p)
            A = zeros(4,4);
            A(1,2) = 1;
            A(2,3) = Quadrotor_MM.g;
            A(3,4) = 1;
            B = zeros(4,1);
            B(4,1) = Quadrotor_MM.len/Quadrotor_MM.EyeX;
            lower = [-2;-2;-pi/2;-pi;-1];
            upper = [2;2;pi/2;pi;1];
            x_init = [px_init,0,0,0];
            x_final = [px_final,0,0,0];
            [~,d_roll_seq] = steerLinearSystembyLP(A,B,lower,upper,x_init,x_final,Quadrotor_MM.dt,n_steps);
        end
        
        function  d_pitch_seq = steerY(py_init,py_final,n_steps)
            % X Control sub-system(states y ydot theta q)
            A = zeros(4,4);
            A(1,2) = 1;
            A(2,3) = -Quadrotor_MM.g;
            A(3,4) = 1;
            B = zeros(4,1);
            B(4,1) = Quadrotor_MM.len/Quadrotor_MM.EyeY;
            lower = [-2;-2;-pi/2;-pi;-1];
            upper = [2;2;pi/2;pi;1];
            x_init = [py_init,0,0,0];
            x_final = [py_final,0,0,0];
            [~,d_pitch_seq] = steerLinearSystembyLP(A,B,lower,upper,x_init,x_final,Quadrotor_MM.dt,n_steps);
        end
    end
end


function [Un,Wg] = generate_control_and_indep_process_noise(U)
% generate Un
indep_part_of_Un = randn(Quadrotor_MM.ctDim,1);
P_Un = control_noise_covariance(U);
Un = indep_part_of_Un.*diag(P_Un.^(1/2));
% generate Wg
Wg = mvnrnd(zeros(Quadrotor_MM.stDim,1),Quadrotor_MM.P_Wg)';
end
function P_Un = control_noise_covariance(U)
u_std=(Quadrotor_MM.eta_u).*U+(Quadrotor_MM.sigma_b_u);
P_Un=diag(u_std.^2);
end


