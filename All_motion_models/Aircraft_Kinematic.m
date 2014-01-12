%% Class Definition
classdef Aircraft_Kinematic < MotionModel_interface
    properties (Constant = true)
        stDim = 7;%state.dim; % state dimension
        ctDim = 4;  % control vector dimension
        wDim = 4;   % Process noise (W) dimension  % For the generality we also consider the additive noise on kinematics equation (3 dimension), but it most probably will set to zero. The main noise is a 2 dimensional noise which is added to the controls.
        dt = user_data_class.par.motion_model_parameters.dt;
        % base_length = user_data_class.par.motion_model_parameters.base_length;  % distance between robot's rear wheels.
        sigma_b_u = [0.02; deg2rad(0.25);deg2rad(0.25); deg2rad(0.25)];%user_data_class.par.motion_model_parameters.sigma_b_u_aircraft ;
        eta_u = [0.005;0.005;0.005;0.005];%user_data_class.par.motion_model_parameters.eta_u_aircraft ;
        P_Wg = [0.2 ; 0.2 ; 0.2 ; 0.1 ; 0.1 ; 0.1 ; 0.1];%user_data_class.par.motion_model_parameters.P_Wg;
        Max_Roll_Rate = deg2rad(45); % try 45
        Max_Pitch_Rate = deg2rad(45);% try 45
        Max_Yaw_Rate = deg2rad(45);% try 45
        Max_Velocity = 8.0; % m/s
        Min_Velocity = 2.5;% m/s
        zeroNoise = zeros(Aircraft_Kinematic.wDim,1);
        turn_radius_min = Aircraft_Kinematic.Min_Velocity/Aircraft_Kinematic.Max_Yaw_Rate; % indeed we need to define the minimum linear velocity in turnings (on orbits) and then find the minimum radius accordingly. But, we picked the more intuitive way.
    end
    
    
    methods (Static = true)
        
        function x_next = f_discrete(x,u,w)
            
            if u(1)> Aircraft_Kinematic.Max_Velocity
                u(1) = Aircraft_Kinematic.Max_Velocity;
            end
            if u(1) <  Aircraft_Kinematic.Min_Velocity
                u(1) = Aircraft_Kinematic.Min_Velocity;
            end
            if u(2)> Aircraft_Kinematic.Max_Roll_Rate
                u(2) = Aircraft_Kinematic.Max_Roll_Rate;
            end
            if u(2) <  -Aircraft_Kinematic.Max_Roll_Rate
                u(2) = -Aircraft_Kinematic.Max_Roll_Rate;
            end
            if u(3)> Aircraft_Kinematic.Max_Pitch_Rate
                u(3) = Aircraft_Kinematic.Max_Pitch_Rate;
            end
            if u(3) <  -Aircraft_Kinematic.Max_Pitch_Rate
                u(3) = -Aircraft_Kinematic.Max_Pitch_Rate;
            end
            if u(4)> Aircraft_Kinematic.Max_Yaw_Rate
                u(4) = Aircraft_Kinematic.Max_Yaw_Rate;
            end
            if u(4) <  -Aircraft_Kinematic.Max_Yaw_Rate
                u(4) = -Aircraft_Kinematic.Max_Yaw_Rate;
            end


            
            pos = [x(1) ; x(2) ; x(3)];% position state
            rot  = [x(4) ; x(5) ; x(6) ; x(7)];% rotation state
            q_rot = Quaternion(rot);
            q_rot = unit(q_rot);% making a normalized quarternion, dont use quatnormalize
            p = q_rot.R;
            
            u_linear_ground  =  p *[u(1); 0; 0] ;
            w_linear_ground = p * [w(1); 0 ; 0];
            x_next_pos = pos + Aircraft_Kinematic.dt * (u_linear_ground) + ((Aircraft_Kinematic.dt^0.5) * w_linear_ground);
            
            u_angular_ground = [u(2); u(3); u(4)];%p * [u(2); u(3); u(4)];
%             u_angular_ground = [u_angular_ground(1); u_angular_ground(2); u_angular_ground(3)];
            w_angular_ground = [w(2); w(3); w(4)]; % p * [w(2); w(3); w(4)]
%             w_angular_ground = [w_angular_ground(1); w_angular_ground(2); w_angular_ground(3)];
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
            
            control = dq_dt_u * Aircraft_Kinematic.dt;
            noise = dq_dt_w * Aircraft_Kinematic.dt^0.5;
            x_next_rot = rot + control + noise;
            q_next = unit(x_next_rot); % Make a unit quaternion
            q_next = correct_quaternion(q_next);
            
            %             %transition_quat = Aircraft_Kinematic.f_transquat(Aircraft_Kinematic.dt , u_angular ,w_angular);
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
            a_12 = - 0.5 * t3(1) * Aircraft_Kinematic.dt - 0.5 * t4(1) * (Aircraft_Kinematic.dt)^0.5;
            a_13 = - 0.5 * t3(2) * Aircraft_Kinematic.dt - 0.5 * t4(2) * (Aircraft_Kinematic.dt)^0.5;
            a_14 = - 0.5 * t3(3) * Aircraft_Kinematic.dt - 0.5 * t4(3) * (Aircraft_Kinematic.dt)^0.5;
            a_21 =   0.5 * t3(1) * Aircraft_Kinematic.dt + 0.5 * t4(1) * (Aircraft_Kinematic.dt)^0.5;
            a_22 =  1 ;
            a_23 = 0.5 * t3(3) * Aircraft_Kinematic.dt + 0.5 * t4(3) * (Aircraft_Kinematic.dt)^0.5;
            a_24 = - 0.5 * t3(2) * Aircraft_Kinematic.dt - 0.5 * t4(2) * (Aircraft_Kinematic.dt)^0.5;
            a_31 = 0.5 * t3(2) * Aircraft_Kinematic.dt + 0.5 * t4(2) * (Aircraft_Kinematic.dt)^0.5;
            a_32 = - 0.5 * t3(3) * Aircraft_Kinematic.dt - 0.5 * t4(3) * (Aircraft_Kinematic.dt)^0.5;
            a_33 = 1 ;
            a_34 = 0.5 * t3(1) * Aircraft_Kinematic.dt + 0.5 * t4(1) * (Aircraft_Kinematic.dt)^0.5;
            a_41 = 0.5 * t3(3) * Aircraft_Kinematic.dt + 0.5 * t4(3) * (Aircraft_Kinematic.dt)^0.5;
            a_42 = 0.5 * t3(2) * Aircraft_Kinematic.dt + 0.5 * t4(2) * (Aircraft_Kinematic.dt)^0.5;
            a_43 = -0.5 * t3(1) * Aircraft_Kinematic.dt - 0.5 * t4(1) * (Aircraft_Kinematic.dt)^0.5;
            a_44 = 1 ;
            A = [a_11 a_12 a_13 a_14; a_21 a_22 a_23 a_24; a_31 a_32 a_33 a_34; a_41 a_42 a_43 a_44];
            % Calculating the jacobian of the linear components B
            qq = q_rot.double; % put it back in double form
            q0 = qq(1);
            q1 = qq(2);
            q2 = qq(3);
            q3 = qq(4);
            Vsum = (u(1)*Aircraft_Kinematic.dt + w(1)*Aircraft_Kinematic.dt^0.5);
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
            
            B = [b_11 b_12 b_13 b_14 b_15 b_16 b_17;b_21 b_22 b_23 b_24 b_25 b_26 b_27;b_31 b_32 b_33 b_34 b_35 b_36 b_37];
            
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
            b_12 = 0.5 *(-q1)* Aircraft_Kinematic.dt ;
            b_13 = 0.5 *(-q2)* Aircraft_Kinematic.dt ;
            b_14 = 0.5 *(-q3)* Aircraft_Kinematic.dt ;
            b_21 = 0 ;
            b_22 = 0.5 *(q0)* Aircraft_Kinematic.dt ;
            b_23 = 0.5 *(-q3)* Aircraft_Kinematic.dt ;
            b_24 = 0.5 *(q2)* Aircraft_Kinematic.dt ;
            b_31 = 0 ;
            b_32 = 0.5 *(q3)* Aircraft_Kinematic.dt ;
            b_33 = 0.5 *(q0)* Aircraft_Kinematic.dt ;
            b_34 = 0.5 *(-q1)* Aircraft_Kinematic.dt ;
            b_41 = 0;
            b_42 = 0.5 *(-q2)* Aircraft_Kinematic.dt ;
            b_43 = 0.5 *(q1) * Aircraft_Kinematic.dt ;
            b_44 = 0.5 *(q0) * Aircraft_Kinematic.dt ;
            B = [b_11 b_12 b_13 b_14; b_21 b_22 b_23 b_24; b_31 b_32 b_33 b_34; b_41 b_42 b_43 b_44];
            % Jacobian calc for linear part
            a_11 = (q0^2+q1^2-q2^2-q3^2) * Aircraft_Kinematic.dt ;
            a_12 = 0;
            a_13 = 0;
            a_14 = 0;
            a_21 = 2*(q1*q2+q0*q3)* Aircraft_Kinematic.dt;
            a_22 = 0;
            a_23 = 0;
            a_24 = 0;
            a_31 = 2*(q1*q3-q0*q2) * Aircraft_Kinematic.dt;
            a_32 = 0;
            a_33 = 0;
            a_34 = 0;
            A = [a_11 a_12 a_13 a_14;a_21 a_22 a_23 a_24;a_31 a_32 a_33 a_34];
            
            J = zeros(7,4);
            J(1:3,1:4) = A;
            J(4:7,1:4) = B;
            
        end
        function J = df_dw_func(x,u,w) % noise Jacobian
            rot  = [x(4) , x(5) , x(6) , x(7)];% rotation state
            q_rot = Quaternion(rot);
            q_rot = unit(q_rot);% making a normalized quarternion
            qq = q_rot.double; % put it back in double form
            R_gb = q_rot.R;
            q0 = qq(1);
            q1 = qq(2);
            q2 = qq(3);
            q3 = qq(4);
            
            g_11 = 0;
            g_12 = 0.5 *(-q1)* sqrt(Aircraft_Kinematic.dt) ;
            g_13 = 0.5 *(-q2)* sqrt(Aircraft_Kinematic.dt) ;
            g_14 = 0.5 *(-q3)* sqrt(Aircraft_Kinematic.dt) ;
            g_21 = 0 ;
            g_22 = 0.5 *(q0)* sqrt(Aircraft_Kinematic.dt);
            g_23 = 0.5 *(-q3)* sqrt(Aircraft_Kinematic.dt);
            g_24 = 0.5 *(q2)* sqrt(Aircraft_Kinematic.dt) ;
            g_31 = 0 ;
            g_32 = 0.5 *(q3)* sqrt(Aircraft_Kinematic.dt) ;
            g_33 = 0.5 *(q0)* sqrt(Aircraft_Kinematic.dt) ;
            g_34 = 0.5 *(-q1)* sqrt(Aircraft_Kinematic.dt) ;
            g_41 = 0;
            g_42 = 0.5 *(-q2)* sqrt(Aircraft_Kinematic.dt) ;
            g_43 = 0.5 *(q1)* sqrt(Aircraft_Kinematic.dt) ;
            g_44 = 0.5 *(q0)* sqrt(Aircraft_Kinematic.dt) ;
            G = [g_11 g_12 g_13 g_14; g_21 g_22 g_23 g_24; g_31 g_32 g_33 g_34; g_41 g_42 g_43 g_44];
            % Jacobian for linear part
            a_11 = (q0^2+q1^2-q2^2-q3^2) * (Aircraft_Kinematic.dt)^0.5 ;
            a_12 = 0;
            a_13 = 0;
            a_14 = 0;
            a_21 = 2*(q1*q2+q0*q3)* (Aircraft_Kinematic.dt)^0.5;
            a_22 = 0;
            a_23 = 0;
            a_24 = 0;
            a_31 = 2*(q1*q3-q0*q2) * (Aircraft_Kinematic.dt)^0.5;
            a_32 = 0;
            a_33 = 0;
            a_34 = 0;
            A = [a_11 a_12 a_13 a_14;a_21 a_22 a_23 a_24;a_31 a_32 a_33 a_34];
            J = zeros(7,4);
            J(1:3,1:4) = A;
            J(4:7,1:4) = G;
            
        end
        function w = generate_process_noise(x,u) % simulate (generate) process noise based on the current poistion and controls
            [Un] =  Aircraft_Kinematic.generate_control_and_indep_process_noise(u);
            w = [Un];
        end
        function [Un] = generate_control_and_indep_process_noise(U)
            % generate Un
            indep_part_of_Un = randn(Aircraft_Kinematic.ctDim,1);
            P_Un =  Aircraft_Kinematic.control_noise_covariance(U);
            Un = indep_part_of_Un.*diag(P_Un.^(1/2));
        end
        function P_Un = control_noise_covariance(U)
            u_std=(Aircraft_Kinematic.eta_u).*U +(Aircraft_Kinematic.sigma_b_u);
            P_Un=diag(u_std.^2);
        end
        function Q_process_noise = process_noise_cov(x,u) % compute the covariance of process noise based on the current poistion and controls
            Q_process_noise = control_noise_covariance(u);
        end
        
        %THERE no validity checking, just implementing for tests sake
        function nominal_traj = generate_VALID_open_loop_point2point_traj(start,goal)
            nominal_traj = Aircraft_Kinematic.generate_open_loop_point2point_traj(start,goal);
        end
        
        function nominal_traj = generate_open_loop_point2point_traj(start,goal) % generates open-loop trajectories between two start and goal states
            % The MATLAB RVC ToolBox Must be loaded first
%             disp('Using RRT3D to connect points on two orbits');
%             disp('Start point is:');start
%             disp('Goal point is:');goal
%             disp('Solving...');
            if is_trajectory_valid(start, goal)
                veh = Aircraft_Kinematic();
                rrt = RRT3D([], veh, 'start', start, 'range', 5,'npoints',500,'speed',2,'time', Aircraft_Kinematic.dt);
                rrt.plan('goal',goal)   ;          % create navigation tree
                nominal_traj.x = [];
                nominal_traj.u = [];
                nominal_traj = rrt.path(start, goal) ; % animate path from this start location
                % Code for plotting
                controls = [];
                nomXs = [];
                for i = 1:length(nominal_traj)
                    p = nominal_traj(i);
                    g  = rrt.graph;
                    data = g.data(p);
                    if ~isempty(data)
                        if i >= length(nominal_traj) || g.edgedir(p, nominal_traj(i+1)) > 0
                            controls = [controls,data.steer'];
                            nomXs = [nomXs,data.path];
                        else
                            controls = [controls,(data.steer(:,end:-1:1))'];
                            nomXs = [nomXs,data.path];

                        end
                    end
                end

                if ~isempty(nomXs)
                    nominal_traj.x = [start,nomXs];
                    nominal_traj.u = controls;
                end

                disp('Done with RRT...');
            else
                nominal_traj =[]; 
                return;
            end
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
                orbit = Aircraft_Kinematic.generate_orbit(orbit_center);
                orbit = Aircraft_Kinematic.draw_orbit(orbit);
            end
            disp('fix the orbit drawing above in aircraft_kinematic.m Line 440')
        end
        %% Construct an orbit
        function orbit = generate_orbit(orbit_center)
            % minimum orbit radius resutls from dividing the minimum linear
            % velocity to maximum angular velocity. However, here we assume
            % that the linear velocity is constant.
            orbit.radius = Aircraft_Kinematic.turn_radius_min;
            orbit_length_meter = 2*pi*orbit.radius;
            orbit_length_time_continuous = orbit_length_meter/Aircraft_Kinematic.Min_Velocity;
            T_rational = orbit_length_time_continuous/Aircraft_Kinematic.dt;
            T = ceil(T_rational);
            orbit.period = T;
            orbit.center = orbit_center;
            
            % defining controls on the orbit
            if T_rational-floor(T_rational)==0
                V_p = Aircraft_Kinematic.Min_Velocity * ones(1,T); % we traverse the orbit with minimum linear velocity
                omega_p = Aircraft_Kinematic.Max_Yaw_Rate * ones(1,T); % we traverse the orbit with maximum angular velocity
            else
                V_p = Aircraft_Kinematic.Min_Velocity * [ones(1,T-1) , T_rational-floor(T_rational)]; % we traverse the orbit with minimum linear velocity
                omega_p = Aircraft_Kinematic.Max_Yaw_Rate * [ones(1,T-1) , T_rational-floor(T_rational)]; % we traverse the orbit with maximum angular velocity
            end
            
            
            u_p = [V_p;zeros(1,T);zeros(1,T);omega_p];
            w_zero = Aircraft_Kinematic.zeroNoise; % no noise
            
            % defining state steps on the orbit
            zeroQuaternion = state.zeroQuaternion; % The robot's angle in the start of orbit is zero
            x_p(:,1) = [orbit_center.val(1:3) - [0;orbit.radius;0] ; zeroQuaternion]; % initial x
            for k=1:T
                x_p(:,k+1) = MotionModel_class.f_discrete(x_p(:,k),u_p(:,k),w_zero);
                tempState = state(x_p(:,k+1));
                if tempState.is_constraint_violated()
                    error('The selected orbit is violating constraints');
                    return
                end
            end
            orbit.x = x_p(:,1:T);  % "x_p" is of length T+1, but "x_p(:,T+1)" is equal to "x_p(:,1)"
            orbit.u = u_p;  % "u_p" is of length T.
            orbit.plot_handle = [];
        end
        function point = point_on_orbit(orbit, point_angle)
            gamma = point_angle;
            point = orbit.radius * [ cos(gamma) ; sin(gamma);0 ] + orbit.center.val(1:3);
            yaw = gamma+pi/2;
            q = angle2quat(yaw,0,0);
            q = correct_quaternion(q);
            point = [point;q'];
            %start_orbit.radius*[cos(gamma_start_of_orbit_edge);sin(gamma_start_of_orbit_edge);0]+start_orbit.center.val;
            
        end
        %% Draw an orbit
        function orbit = draw_orbit(orbit,varargin)
            % This function draws the orbit.
            % default values
            orbit_color = 'b'; % Default value for "OrbitTextColor" property. % User-provided value for "OrbitTextColor" property.
            orbit_width = 2; % User-provided value for "orbit_width" property. % User-provided value for shifting the text a little bit to the left. % for some reason MATLAB shifts the starting point of the text a little bit to the right. So, here we return it back.
            robot_shape = 'triangle'; % The shape of robot (to draw trajectories and to show direction of edges and orbits)
            robot_size = 1; % Robot size on orbits (to draw trajectories and to show direction of edges and orbits)
            orbit_trajectory_flag = 0; % Make it one if you want to see the orbit trajectories. Zero, otherwise.
            text_size = 12;
            text_color = 'b';
            text_shift = 0.8;
            orbit_text = [];
            
            % parsing the varargin
            if ~isempty(varargin)
                for i = 1 : 2 : length(varargin)
                    switch lower(varargin{i})
                        case lower('RobotSize')
                            robot_size = varargin{i+1};
                        case lower('OrbitWidth')
                            orbit_width = varargin{i+1};
                        case lower('OrbitColor')
                            orbit_color = varargin{i+1};
                        case lower('OrbitText')
                            orbit_text = varargin{i+1};
                    end
                end
            end
            % start drawing
            if orbit_trajectory_flag == 1
                orbit.plot_handle = [];
                for k=1:orbit.period
                    Xstate = state(orbit.x(:,k));
                    Xstate = Xstate.draw('RobotShape',robot_shape,'robotsize',robot_size);
                    orbit.plot_handle = [orbit.plot_handle,Xstate.head_handle,Xstate.text_handle,Xstate.tria_handle];
                end
                tmp = plot(orbit.x(1,:) , orbit.x(2,:),orbit_color);
                orbit.plot_handle = [orbit.plot_handle,tmp];
            else
                orbit.plot_handle = [];
                th_orbit_draw = [0:0.1:2*pi , 2*pi];
                x_orbit_draw = orbit.center.val(1) + orbit.radius*cos(th_orbit_draw);
                y_orbit_draw = orbit.center.val(2) + orbit.radius*sin(th_orbit_draw);
                z_orbit_draw = orbit.center.val(3)*ones(1,size(x_orbit_draw,2));
                tmp_h = plot3(x_orbit_draw,y_orbit_draw,z_orbit_draw,'lineWidth',orbit_width);
                Xstate = state(orbit.x(:,1));
                Xstate = Xstate.draw('RobotShape',robot_shape,'robotsize',robot_size);
                orbit.plot_handle = [orbit.plot_handle,tmp_h,Xstate.head_handle,Xstate.text_handle,Xstate.tria_handle];
            end
            
            if ~isempty(orbit_text)
                text_pos = orbit.center.val;
                text_pos(1) = text_pos(1) - text_shift; % for some reason MATLAB shifts the starting point of the text a little bit to the right. So, here we return it back.
                tmp_handle = text( text_pos(1), text_pos(2), orbit_text, 'fontsize', text_size, 'color', text_color);
                orbit.plot_handle = [orbit.plot_handle,tmp_handle];
            end
            
        end
        function YesNo = is_constraints_violated(open_loop_traj)
            error('not yet implemented');
        end
        function traj_plot_handle = draw_nominal_traj(nominal_traj, traj_flag, varargin)
            traj_plot_handle = [];
            if traj_flag == 1
                for k = 1 : size(nominal_traj.x , 2)
                    tmp_Xstate = state (nominal_traj.x(:,k) );
                    tmp_Xstate.draw();
                    % tmp_Xstate.draw('RobotShape','triangle','robotsize',1);%,'TriaColor',color(cycles));
                    %traj_plot_handle(k:k+2) =
                    %[tmp_Xstate.head_handle,tmp_Xstate.text_handle,tmp_Xstate.tria_handle];
                end
            elseif traj_flag == 2
                tmp_handle = plot3(nominal_traj.x(1,:) , nominal_traj.x(2,:) , nominal_traj.x(3,:), 'LineWidth',4, 'color','g');
%                 traj_plot_handle = [traj_plot_handle , tmp_handle];
                len = size( nominal_traj.x , 2);
                [yaw,pitch,roll] = quat2angle(nominal_traj.x(4:7,floor(len/2))');
                pitch =0;
                roll =0;
                new_quat = angle2quat(yaw,pitch,roll);
                tmp_Xstate = state( [nominal_traj.x(1:3,floor(len/2)) ; new_quat']); % to plot the direction of the line.
                tmp_Xstate = tmp_Xstate.draw('RobotShape','triangle','robotsize',2);
                traj_plot_handle = [traj_plot_handle , tmp_Xstate.plot_handle , tmp_Xstate.head_handle , tmp_Xstate.tria_handle , tmp_Xstate.text_handle ];
                drawnow
            elseif traj_flag == 3
                tmp_handle = plot3(nominal_traj.x(1,:) , nominal_traj.x(2,:) , nominal_traj.x(3,:), 'LineWidth',4, 'color','r');
                %                 traj_plot_handle = [traj_plot_handle , tmp_handle];
                len = size( nominal_traj.x , 2);
                [yaw,pitch,roll] = quat2angle(nominal_traj.x(4:7,floor(len/2))');
                pitch =0;
                roll =0;
                new_quat = angle2quat(yaw,pitch,roll);
                tmp_Xstate = state( [nominal_traj.x(1:3,floor(len/2)) ; new_quat']); % to plot the direction of the line.
                tmp_Xstate = tmp_Xstate.draw('RobotShape','triangle','robotsize',2);
                traj_plot_handle = [traj_plot_handle , tmp_Xstate.plot_handle , tmp_Xstate.head_handle , tmp_Xstate.tria_handle , tmp_Xstate.text_handle ];
                drawnow
            else
                tmp_handle = plot3(nominal_traj.x(1,:) , nominal_traj.x(2,:) , nominal_traj.x(3,:), 'LineWidth',2);
%                 traj_plot_handle = [traj_plot_handle , tmp_handle];
                len = size( nominal_traj.x , 2);
                [yaw,pitch,roll] = quat2angle(nominal_traj.x(4:7,floor(len/2))');
                pitch =0;
                roll =0;
                new_quat = angle2quat(yaw,pitch,roll);
                tmp_Xstate = state( [nominal_traj.x(1:3,floor(len/2)) ; new_quat']); % to plot the direction of the line.
                tmp_Xstate = tmp_Xstate.draw('RobotShape','triangle','robotsize',2);
                traj_plot_handle = [traj_plot_handle , tmp_Xstate.plot_handle , tmp_Xstate.head_handle , tmp_Xstate.tria_handle , tmp_Xstate.text_handle ];
                drawnow
            end
        end
        function plot_handle = draw_orbit_neighborhood(orbit, scale)
            % not implemented yet
            plot_handle = [];
        end
    end
end

function qcorrected = correct_quaternion(q)
    qcorrected = q;
    if q(1) < 0
        qcorrected = -q;
    end
end

function YesNo = is_trajectory_valid(start, goal)

    vel = (Aircraft_Kinematic.Min_Velocity + Aircraft_Kinematic.Max_Velocity)/2;
    nPointsOnSegment = ceil(( norm(goal(1:3)-start(1:3)) )/(vel*Aircraft_Kinematic.dt));
    
    guidanceVector = goal(1:3) - start(1:3);

    for i=1:nPointsOnSegment+1
        point = start(1:3) + (i/(nPointsOnSegment+1))*guidanceVector(1:3);
        tmp = state([point;1;0;0;0]); 
        if tmp.is_constraint_violated 
            YesNo=0; 
            return; 
        end
    end
    YesNo = 1;
end
%% Generating Control-dependent Noise Covariance
%
% $$ P^{U_n} = \left(
%   \begin{array}{cc}
%     (\eta_V V + \sigma_{b_V})^2 & 0\\
%     0 & (\eta_{\omega} \omega + \sigma_{b_{\omega}})^2\\
%   \end{array}\right) $$,
%
% where, $\eta_u=(\eta_V,\eta_{\omega})^T$ and
% $\sigma_{b_u}=(\sigma_{b_V},\sigma_{b_{\omega}})^T$.

function P_Un = control_noise_covariance(U)
u_std = (Aircraft_Kinematic.eta_u).*U+(Aircraft_Kinematic.sigma_b_u);
P_Un  = diag(u_std.^2);
end