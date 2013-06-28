classdef RandomWalk < MotionModel_interface
    % Note that because the class is defined as a handle class, the
    % properties must be defined such that they are do not change from an
    % object to another one.
    properties (Constant)
        stDim = state.dim; % state dimension
        ctDim = state.dim;  % control vector dimension
        wDim = state.dim;   % Process noise (W) dimension
        dt = user_data_class.par.motion_model_parameters.dt;
        sigma_b_u = 0; % At this point, we are not using this varaible. We should use it later to generalize the model. % A constant bias intensity (covariance) of the control noise
        eta_u = 0; % At this point, we are not using this varaible. We should use it later to generalize the model. % A coefficient, which makes the control noise intensity proportional to the control signal
        P_Wg = user_data_class.par.motion_model_parameters.P_Wg_team;
        velocity_max = 0.2*user_data_class.par.motion_model_parameters.V_const_path_team; % Note that "V_const_path_team" is a vector and so is the "velocity_max"
    end
    
    methods (Static)
        function x_next = f_discrete(x,u,w)
            x_next = x+u*RandomWalk.dt+w*sqrt(RandomWalk.dt);
        end
        function A = df_dx_func(x,u,w) %#ok<INUSD>
            A = eye(RandomWalk.stDim);
        end
        function B = df_du_func(x,u,w) %#ok<INUSD>
            B = eye(RandomWalk.stDim)*RandomWalk.dt;
        end
        function G = df_dw_func(x,u,w) %#ok<INUSD>
            G = eye(RandomWalk.stDim)*sqrt(RandomWalk.dt);
        end
        function Q_process_noise = process_noise_cov(x,u) %#ok<INUSD>
            Q_process_noise = RandomWalk.P_Wg;
        end
        function w = generate_process_noise(x,u) %#ok<INUSD>
            w = mvnrnd(zeros(RandomWalk.stDim,1),RandomWalk.P_Wg)';
        end

        function nominal_traj = generate_VALID_open_loop_point2point_traj(X_initial,X_final) % generates open-loop trajectories between two start and goal states
            if isa(X_initial,'state'), X_initial=X_initial.val; end % retrieve the value of the state vector
            if isa(X_final,'state'), X_final=X_final.val; end % retrieve the value of the state vector
            % parameters
            dt_local = RandomWalk.dt;
            stDim_local = RandomWalk.stDim;
            
            X_diff_signed = X_final - X_initial;  % I am not sure if I should use the "subtraction" of the "state class" here or just subtract them directly.
            vector_of_times = abs(X_diff_signed)./RandomWalk.velocity_max;
            max_time = max(vector_of_times);
            
            %--------------Total number of steps
            kf_rational = max_time/dt_local;
            kf = floor(kf_rational)+1;  % note that in all following lines you cannot replace "floor(something)+1" by "ceil(something)", as it may be  a whole number.
            
            new_velocities = RandomWalk.velocity_max .* sign(X_diff_signed) .* (vector_of_times /max_time);
            
            delta_state_const = new_velocities * dt_local; % the fixed velocities along the path, which result in the fixed state increments.
            delta_state_nominal_seq( : , 1:kf-1 ) = repmat( delta_state_const , 1 , kf-1); % The velocity profile (sequence of velocities)
            delta_state_const_end = new_velocities*dt_local*(kf_rational - floor(kf_rational));  % The reduced velocity in the last step
            delta_state_nominal_seq( : , kf ) = delta_state_const_end;
            
             %=====================Nominal control and state trajectory generation
            u_p = delta_state_nominal_seq/dt_local;
            
            x_p = zeros(stDim_local,kf+1);
            x_p(:,1) = X_initial;
            for k = 1:kf
                x_p(:,k+1) = x_p(:,k) + delta_state_nominal_seq(:,k);
                tmp = state(x_p(:,k+1)); if tmp.is_constraint_violated, nominal_traj =[]; return; end
            end
            
            % noiselss motion  % for debug: if you uncomment the following
            % lines you have to get the same "x_p_copy" as the "x_p"
            %             x_p_copy = zeros(stDim,kf+1);
            %             x_p_copy(:,1) = X_initial;
            %             for k = 1:kf
            %                 x_p_copy(:,k+1) = RandomWalk.f_discrete(x_p_copy(:,k),u_p(:,k),zeros(RandomWalk.wDim,1));
            %             end
            
            nominal_traj.x = x_p;
            nominal_traj.u = u_p;
        end
        function YesNo = is_constraints_violated(open_loop_traj) % this function checks if the "open_loop_traj" violates any constraints or not. For example it checks collision with obstacles.
            % In this class the open loop trajectories are indeed straight
            % lines. So, we use following simplified procedure to check the
            % collisions.
            if state.dim ~= 6
                error('This function is optimized for the three 2D robots');
            end
            YesNo = 0;
            Obst=obstacles_class.obst;
            for ir = 1:state.num_robots
                edge_start = open_loop_traj.x(2*ir-1:2*ir , 1);
                edge_end = open_loop_traj.x(2*ir-1:2*ir , end);
                
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
        function traj_plot_handle = draw_nominal_traj(nominal_traj, varargin)
            if state.dim ~= 6
                error('This function is optimized for the three 2D robots');
            end
            for ir = 1:state.num_robots
                s_node_2D_loc = nominal_traj.x(2*ir-1:2*ir , 1);
                e_node_2D_loc = nominal_traj.x(2*ir-1:2*ir , end);
                % retrieve PRM parameters provided by the user
                %             disp('the varargin need to be parsed here')
                %             edge_spec = obj.par.edge_spec;
                %             edge_width = obj.par.edge_width;
                edge_spec = '-b';
                edge_width = 2;
                
                % drawing the 2D edge line
                traj_plot_handle = plot([s_node_2D_loc(1),e_node_2D_loc(1)],[s_node_2D_loc(2),e_node_2D_loc(2)],edge_spec,'linewidth',edge_width);
            end
        end
    end

end