%% Class Definition
classdef Aircraft_Kinematic < MotionModel_interface
    properties (Constant = true)
        stDim = state.dim; % state dimension
        ctDim = 4;  % control vector dimension
        wDim = 4;   % Process noise (W) dimension  % For the generality we also consider the additive noise on kinematics equation (3 dimension), but it most probably will set to zero. The main noise is a 2 dimensional noise which is added to the controls.
        dt = 0.3;
        % base_length = user_data_class.par.motion_model_parameters.base_length;  % distance between robot's rear wheels.
        sigma_b_u = user_data_class.par.motion_model_parameters.sigma_b_u_aircraft ;
        eta_u = user_data_class.par.motion_model_parameters.eta_u_aircraft ;
        P_Wg = user_data_class.par.motion_model_parameters.P_Wg;
        Max_Roll_Rate = deg2rad(45); % try 45
        Max_Pitch_Rate = deg2rad(45);% try 45
        Max_Yaw_Rate = deg2rad(45);% try 45
        Max_Velocity = 1.5; % m/s
        Min_Velocity = 0.5;% m/s
        zeroNoise = zeros(Aircraft_Kinematic.wDim,1);
    end
    
    properties (Constant = true) % orbit-related properties
        turn_radius_min = 1.5*0.1; % indeed we need to define the minimum linear velocity in turnings (on orbits) and then find the minimum radius accordingly. But, we picked the more intuitive way.
        angular_velocity_max = deg2rad(45); % degree per second (converted to radian per second)
        linear_velocity_min_on_orbit = Unicycle_robot.turn_radius_min*Unicycle_robot.angular_velocity_max; % note that on the straight line the minimum velocity can go to zero. But, in turnings (on orbit) the linear velocity cannot fall below this value.
        linear_velocity_max =1.5;
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
        %           function transition_quat = f_transquat(dt , u ,w) % to calculate transition quaternion
        %             u_res = u + w; % noisy input
        %             u_bar = Quaternion(u_res);
        %             u_mod = norm(u_bar);%making a normalized quaternion
        %             q_scalar = cos((u_mod * dt)/2);
        %             q_common = sin((u_mod * dt)/2)/u_mod;
        %             q_vector = u_res * q_common;
        %             transition_quat = Quaternion(q_scalar,q_vector);%returning transition quaternion
        %           end
        %           function x_next = f_discrete(x,u,w)
        %             V = u(1);
        %             l = u(2);
        %             q = u(3);
        %             r = u(4);
        %             pos = [x(1) , x(2) , x(3)];% position state
        %             rot = [x(4) , x(5) , x(6) , x(7)];% rotation state
        %             q_rot = Quaternion(rot);
        %             q_rot = unit(q_rot);% making a normalized quarternion
        %             p = q_rot.R;
        %             t1 = p *[V; 0; 0];
        %             u_linear  =  [t1(1) t1(2) t1(3)];
        %             u_angular = [u(2) u(3) u(4)];
        %             linear_noise    = p * [w(1); 0 ; 0];
        %             w_linear = [linear_noise(1) linear_noise(2) linear_noise(3)];
        %             w_angular = [w(2) w(3) w(4)];
        %             transition_quat = Aircraft_Kinematic.f_transquat(Aircraft_Kinematic.dt , u_angular ,w_angular);
        %             x_next_q_rot = Quaternion();
        %             x_next_q_rot = unit(q_rot * transition_quat);% normalizing resultant quaternion)
        %             x_next_rot = double(x_next_q_rot);% converting to matrix form
        %             x_next_pos = pos + Aircraft_Kinematic.dt * (u_linear) + ((Aircraft_Kinematic.dt^0.5) * w_linear);
        %             x_next = [x_next_pos, x_next_rot]; % augmenting state and rotational part
        % %         end
        function x_next = f_discrete(x,u,w)
            pos = [x(1) ; x(2) ; x(3)];% position state
            rot  = [x(4) ; x(5) ; x(6) ; x(7)];% rotation state
            q_rot = Quaternion(rot);
            q_rot = unit(q_rot);% making a normalized quarternion, dont use quatnormalize
            p = q_rot.R;
            
            u_linear_ground  =  p *[u(1); 0; 0] ;
            w_linear_ground = p * [w(1); 0 ; 0];
            x_next_pos = pos + Aircraft_Kinematic.dt * (u_linear_ground) + ((Aircraft_Kinematic.dt^0.5) * w_linear_ground);

            u_angular_ground = p * [u(2); u(3); u(4)];
            u_angular_ground = [0 ;u_angular_ground(1); u_angular_ground(2); u_angular_ground(3)];
            w_angular_ground = p * [w(2); w(3); w(4)];
            w_angular_ground = [0 ; w_angular_ground(1); w_angular_ground(2); w_angular_ground(3)];
            dq_dt = [x(4), -x(5),  -x(6), -x(7);
                 x(5),  x(4)  , -x(7), x(6);
                 x(6) , x(7)  , x(4),  -x(5);
                 x(7) , -x(6),  x(5) ,  x(4)];
            
            time_control = [0.5* Aircraft_Kinematic.dt 0 0 0;
                   0  0.5* Aircraft_Kinematic.dt 0 0 ;
                   0 0 0.5* Aircraft_Kinematic.dt 0 ;
                   0 0 0 0.5* Aircraft_Kinematic.dt];
            time_noise = [ 0.5 * (Aircraft_Kinematic.dt^ 0.5) 0 0 0;
                     0 0.5 * (Aircraft_Kinematic.dt^ 0.5) 0 0 ;
                     0 0 0.5 * (Aircraft_Kinematic.dt^ 0.5) 0;
                     0 0 0 0.5 * (Aircraft_Kinematic.dt^ 0.5)];
            a_con = time_control * dq_dt;
            a_noi = time_noise * dq_dt;
            control = a_con * u_angular_ground;
            noise = a_noi * w_angular_ground;
            x_next_rot = rot + control + noise;
            q_next = unit(x_next_rot); % Make a unit quaternion
            
            %             %transition_quat = Aircraft_Kinematic.f_transquat(Aircraft_Kinematic.dt , u_angular ,w_angular);
            %             x_next_q_rot = Quaternion();
            %             x_next_q_rot = unit(q_rot * transition_quat);% normalizing resultant quaternion)
            %             x_next_rot = double(x_next_q_rot);% converting to matrix f
          
            x_next = [x_next_pos(1)  x_next_pos(2)  x_next_pos(3) q_next(1) q_next(2) q_next(3) q_next(4)]'; % augmenting state and rotational part
        end
%         function  A = df_dx_func(x,u,w) % state Jacobian
%             u_linear =  [u(1) u(2) u(3)];
%             u_angular = [u(4) u(5) u(6)];
%             w_linear = [w(1) w(2) w(3)];
%             w_angular = [w(4) w(5) w(6)];
%             transition_quat = Aircraft_Kinematic.f_transquat(Aircraft_Kinematic.dt , u_angular ,w_angular);
%             transition_mat = double(transition_quat);
%             a_11 = transition_mat(1);
%             a_12 = -1 * transition_mat(2);
%             a_13 = -1 * transition_mat(3);
%             a_14 = -1 * transition_mat(4);
%             a_21 = transition_mat(2);
%             a_22 = transition_mat(1);
%             a_23 = transition_mat(4);
%             a_24 = -1 * transition_mat(3);
%             a_31 = transition_mat(3);
%             a_32 = -1 * transition_mat(4);
%             a_33 = transition_mat(1);
%             a_34 = transition_mat(2);
%             a_41 = transition_mat(4);
%             a_42 = transition_mat(3);
%             a_43 = -1 * transition_mat(2);
%             a_44 = transition_mat(1);
%             A = [a_11 a_12 a_13 a_14; a_21 a_22 a_23 a_24; a_31 a_32 a_33 a_34; a_41 a_42 a_43 a_44];
%         end
         function  J = df_dx_func(x,u,w) % state Jacobian
            pos = [x(1) , x(2) , x(3)];% position state
            rot  = [x(4) , x(5) , x(6) , x(7)];% rotation state
            q_rot = Quaternion(rot);
            q_rot = unit(q_rot);% making a normalized quarternion
            p = q_rot.R;
            u_angular = [u(2); u(3); u(4)];
            t3 = p * u_angular;
            u_angular_ground = [0 ;t3(1); t3(2); t3(3)];
            w_angular = [w(2); w(3); w(4)];
            t4 = p * w_angular;
            w_angular_ground = [0 ; t4(1); t4(2); t4(3)];
            a_11 = 1 + 0.5 * 0 * Aircraft_Kinematic.dt + 0.5 * 0 * (Aircraft_Kinematic.dt)^0.5;
            a_12 = 0 - 0.5 * t3(1) * Aircraft_Kinematic.dt - 0.5 * t4(1) * (Aircraft_Kinematic.dt)^0.5;
            a_13 = 0 - 0.5 * t3(2) * Aircraft_Kinematic.dt - 0.5 * t4(2) * (Aircraft_Kinematic.dt)^0.5;
            a_14 = 0 - 0.5 * t3(3) * Aircraft_Kinematic.dt - 0.5 * t4(3) * (Aircraft_Kinematic.dt)^0.5;
            a_21 = 0 + 0.5 * t3(1) * Aircraft_Kinematic.dt + 0.5 * t4(1) * (Aircraft_Kinematic.dt)^0.5;
            a_22 = 1 + 0.5 * 0 * Aircraft_Kinematic.dt + 0.5 * 0 * (Aircraft_Kinematic.dt)^0.5;
            a_23 = 0 + 0.5 * t3(3) * Aircraft_Kinematic.dt + 0.5 * t4(3) * (Aircraft_Kinematic.dt)^0.5;
            a_24 = 0 - 0.5 * t3(2) * Aircraft_Kinematic.dt - 0.5 * t4(2) * (Aircraft_Kinematic.dt)^0.5;
            a_31 = 0 + 0.5 * t3(2) * Aircraft_Kinematic.dt + 0.5 * t4(2) * (Aircraft_Kinematic.dt)^0.5;
            a_32 = 0 - 0.5 * t3(3) * Aircraft_Kinematic.dt - 0.5 * t4(3) * (Aircraft_Kinematic.dt)^0.5;
            a_33 = 1 + 0.5 * 0 * Aircraft_Kinematic.dt + 0.5 * 0 * (Aircraft_Kinematic.dt)^0.5;
            a_34 = 0 + 0.5 * t3(1) * Aircraft_Kinematic.dt + 0.5 * t4(1) * (Aircraft_Kinematic.dt)^0.5;
            a_41 = 0 + 0.5 * t3(3) * Aircraft_Kinematic.dt + 0.5 * t4(3) * (Aircraft_Kinematic.dt)^0.5;  
            a_42 = 0 + 0.5 * t3(2) * Aircraft_Kinematic.dt + 0.5 * t4(2) * (Aircraft_Kinematic.dt)^0.5;
            a_43 = 0 - 0.5 * t3(1) * Aircraft_Kinematic.dt - 0.5 * t4(1) * (Aircraft_Kinematic.dt)^0.5;
            a_44 = 1 + 0.5 * 0 * Aircraft_Kinematic.dt + 0.5 * 0 * (Aircraft_Kinematic.dt)^0.5;
            A = [a_11 a_12 a_13 a_14; a_21 a_22 a_23 a_24; a_31 a_32 a_33 a_34; a_41 a_42 a_43 a_44];
            % Calculating the jacobian of the linear components B
            qq = q_rot.double; % put it back in double form
            Vsum = (u(1)*Aircraft_Kinematic.dt + w(1)*Aircraft_Kinematic.dt^0.5);
            b_11 = 1;
            b_12 = 0;
            b_13 = 0;
            b_14 = 2*qq(1)  * Vsum;
            b_15 = 2*qq(2)  * Vsum;
            b_16 = -2*qq(3) * Vsum;
            b_17 = -2*qq(4) * Vsum;
            b_21 = 0;
            b_22 = 1;
            b_23 = 0;
            b_24 = qq(4)*Vsum;
            b_25 = 2*qq(3)*Vsum;
            b_26 = 2*qq(2)*Vsum;
            b_27 = qq(1)*Vsum;
            b_31 = 0;
            b_32 = 0;
            b_33 = 1;
            b_34 = -qq(3)*Vsum;
            b_35 = 2*qq(4)*Vsum;
            b_36 = -qq(1)*Vsum;
            b_37 = 2*qq(2)*Vsum;
            
            B = [b_11 b_12 b_13 b_14 b_15 b_16 b_17;b_21 b_22 b_23 b_24 b_25 b_26 b_27;b_31 b_32 b_33 b_34 b_35 b_36 b_37];
            
            J = zeros(7,7);
            J(1:3,1:7) = B;
            J(4:7,4:7) = A; 
           
         end
%         function a1 = j1(rot,p,p5,noisy)
%             a1 = rot* p * p5 * noisy;
%         end
%         function a2 = j2(rot,p ,p5,noisy_1, u_mod, noisy_2)
%             a2 =  rot * noisy_1 * (1/u_mod) * p *p5 * noisy_2 ;
%         end
%         function a3 = j3(rot, p, noisy,u_mod)
%             a3 = rot * p * (1/u_mod);
%         end
%         function a4 = j4(rot, p, noisy_1 , u_mod, noisy_2)
%             a4 =  rot * p * noisy_1 * (-1/2) * 2 * noisy_2 * ((1/u_mod)^(3/2));
%         end
%         function B = df_du_func(x,u,w) % control Jacobian
%             u_angular = [u(4) u(5) u(6)];
%             w_angular = [w(4) w(5) w(6)];
%             u_res = u_angular + w_angular ; % noisy input
%             u_bar = Quaternion(u_res);
%             u_mod = norm(u_bar);%making a normalized quaternion
%             rot = [x(4) , x(5) , x(6) , x(7)];% rotation state
%             noisy_x = u_angular(1) + w_angular(1);
%             noisy_y = u_angular(2) + w_angular(2);
%             noisy_z = u_angular(3) + w_angular(3);
%             p1 =cos((u_mod * dt)/2);
%             p2 = sin((u_mod * dt)/2);
%             p3 = p2 * (dt/2);
%             p4 = p1 * (dt/2);
%             p5 = 1/2 * 2 * (1/u_mod) ;
%             
%             b_11 = - j1(rot(1) ,p3 , p5 , noisy_x) - j2(rot(2), p4, p5,noisy_x, u_mod, noisy_x) - j3(rot(2), p2, noisy_x, u_mod) - j4(rot(2), p2, noisy_x , u_mod, noisy_x) - j2(rot(3), p4, noisy_y, u_mod , noisy_x) - j4(rot(3),p2,noisy_y,u_mod, noisy_x) - j2(rot(4), p4, noisy_z, u_mod , noisy_x) - j4(rot(4),p2,noisy_z,u_mod, noisy_x);
%             b_12 = - j1(rot(1) ,p3 , p5 , noisy_y) - j2(rot(3), p4, p5,noisy_y, u_mod, noisy_y) - j3(rot(3), p2, noisy_y, u_mod) - j4(rot(3), p2, noisy_y , u_mod, noisy_y) - j2(rot(2), p4, noisy_x, u_mod , noisy_y) - j4(rot(2),p2,noisy_x,u_mod, noisy_y) - j2(rot(4), p4, noisy_z, u_mod , noisy_y) - j4(rot(4),p2,noisy_z,u_mod, noisy_y);
%             b_13 = - j1(rot(1) ,p3 , p5 , noisy_z) - j2(rot(4), p4, p5,noisy_z, u_mod, noisy_z) - j3(rot(4), p2, noisy_z, u_mod) - j4(rot(4), p2, noisy_z , u_mod, noisy_z) - j2(rot(2), p4, noisy_x, u_mod , noisy_z) - j4(rot(2),p2,noisy_x,u_mod, noisy_z) - j2(rot(3), p4, noisy_y, u_mod , noisy_z) - j4(rot(3),p2,noisy_y,u_mod, noisy_z);
%             b_21 = - j1(rot(2) ,p3 , p5 , noisy_x) + j2(rot(1) ,p4 ,p5,noisy_x, u_mod, noisy_x) + j3(rot(1), p2, noisy_x, u_mod) + j4(rot(1), p2, noisy_x , u_mod, noisy_x) - j2(rot(4), p4, noisy_y, u_mod , noisy_x) - j4(rot(4),p2,noisy_y,u_mod, noisy_x) + j2(rot(3), p4, noisy_z, u_mod , noisy_x) + j4(rot(3),p2,noisy_z,u_mod, noisy_x);
%             b_22 = - j1(rot(2) ,p3 , p5 , noisy_y) - j2(rot(4) ,p4 ,p5,noisy_y, u_mod, noisy_y) - j3(rot(4), p2, noisy_y, u_mod) - j4(rot(4), p2, noisy_y , u_mod, noisy_y) + j2(rot(1), p4, noisy_x, u_mod , noisy_y) + j4(rot(1),p2,noisy_x,u_mod, noisy_y) + j2(rot(3), p4, noisy_z, u_mod , noisy_y) + j4(rot(3),p2,noisy_z,u_mod, noisy_y);
%             b_23 = - j1(rot(2) ,p3 , p5 , noisy_z) + j2(rot(3) ,p4 ,p5,noisy_z, u_mod, noisy_z) + j3(rot(3), p2, noisy_z, u_mod) + j4(rot(3), p2, noisy_z , u_mod, noisy_z) + j2(rot(1), p4, noisy_x, u_mod , noisy_z) + j4(rot(1),p2,noisy_x,u_mod, noisy_z) - j2(rot(4), p4, noisy_y, u_mod , noisy_z) - j4(rot(4),p2,noisy_y,u_mod, noisy_z);
%             b_31 = - j1(rot(3) ,p3 , p5 , noisy_x) + j2(rot(4) ,p4 ,p5,noisy_x, u_mod, noisy_x) + j3(rot(4), p2, noisy_x, u_mod) + j4(rot(4), p2, noisy_x , u_mod, noisy_x) + j2(rot(1), p4, noisy_y, u_mod , noisy_x) + j4(rot(1),p2,noisy_y,u_mod, noisy_x) - j2(rot(2), p4, noisy_z, u_mod , noisy_x) - j4(rot(2),p2,noisy_z,u_mod, noisy_x);
%             b_32 = - j1(rot(3) ,p3 , p5 , noisy_y) + j2(rot(1) ,p4 ,p5,noisy_y, u_mod, noisy_y) + j3(rot(1), p2, noisy_y, u_mod) + j4(rot(1), p2, noisy_y , u_mod, noisy_y) + j2(rot(4), p4, noisy_x, u_mod , noisy_y) + j4(rot(4),p2,noisy_x,u_mod, noisy_y) - j2(rot(2), p4, noisy_z, u_mod , noisy_y) - j4(rot(2),p2,noisy_z,u_mod, noisy_y);
%             b_33 = - j1(rot(3) ,p3 , p5 , noisy_z) - j2(rot(2) ,p4 ,p5,noisy_z, u_mod, noisy_z) - j3(rot(2), p2, noisy_z, u_mod) - j4(rot(2), p2, noisy_z , u_mod, noisy_z) + j2(rot(4), p4, noisy_x, u_mod , noisy_z) + j4(rot(4),p2,noisy_x,u_mod, noisy_z) + j2(rot(1), p4, noisy_y, u_mod , noisy_z) + j4(rot(1),p2,noisy_y,u_mod, noisy_z);
%             b_41 = - j1(rot(3) ,p3 , p5 , noisy_x) - j2(rot(3) ,p4 ,p5,noisy_x, u_mod, noisy_x) - j3(rot(3), p2, noisy_x, u_mod) - j4(rot(3), p2, noisy_x , u_mod, noisy_x) + j2(rot(2), p4, noisy_y, u_mod , noisy_x) + j4(rot(2),p2,noisy_y,u_mod, noisy_x) + j2(rot(1), p4, noisy_z, u_mod , noisy_x) + j4(rot(1),p2,noisy_z,u_mod, noisy_x);
%             b_42 = - j1(rot(3) ,p3 , p5 , noisy_y) + j2(rot(2) ,p4 ,p5,noisy_y, u_mod, noisy_y) + j3(rot(2), p2, noisy_y, u_mod) + j4(rot(2), p2, noisy_y , u_mod, noisy_y) - j2(rot(3), p4, noisy_x, u_mod , noisy_y) - j4(rot(3),p2,noisy_x,u_mod, noisy_y) + j2(rot(1), p4, noisy_z, u_mod , noisy_y) + j4(rot(1),p2,noisy_z,u_mod, noisy_y);
%             b_43 = - j1(rot(3) ,p3 , p5 , noisy_z) + j2(rot(1) ,p4 ,p5,noisy_z, u_mod, noisy_z) + j3(rot(1), p2, noisy_z, u_mod) + j4(rot(1), p2, noisy_z , u_mod, noisy_z) - j2(rot(3), p4, noisy_x, u_mod , noisy_z) - j4(rot(3),p2,noisy_x,u_mod, noisy_z) + j2(rot(2), p4, noisy_y, u_mod , noisy_z) + j4(rot(2),p2,noisy_y,u_mod, noisy_z);
%             B  = [b_11 b_12 b_13 ; b_21 b_22 b_23 ; b_31 b_32 b_33; b_41 b_42 b_43];
%         end
        function J = df_du_func(x,u,w)
            pos = [x(1) , x(2) , x(3)];% position state
            rot  = [x(4) , x(5) , x(6) , x(7)];% rotation state
            q_rot = Quaternion(rot);
            q_rot = unit(q_rot);% making a normalized quarternion
%             p = q_rot.R;
%             u_angular = [u(2); u(3); u(4)];
%             t3 = p * u_angular;
%             u_angular_ground = [0 ;t3(1); t3(2); t3(3)];
%             w_angular = [w(2); w(3); w(4)];
%             t4 = p * w_angular;
%             w_angular_ground = [0 ; t4(1); t4(2); t4(3)];
            b_11 = 0;
            b_12 = -1 * 0.5 * x(5) * Aircraft_Kinematic.dt ;
            b_13 = -1 * 0.5 * x(6) * Aircraft_Kinematic.dt ;
            b_14 = -1 * 0.5 * x(7) * Aircraft_Kinematic.dt ;
            b_21 = 0 ;
            b_22 = 0.5 * x(4) * Aircraft_Kinematic.dt;
            b_23 = -1 * 0.5 * x(7) * Aircraft_Kinematic.dt ;
            b_24 = 0.5 * x(6) * Aircraft_Kinematic.dt ;
            b_31 = 0 ;
            b_32 = 0.5 * x(7) * Aircraft_Kinematic.dt;
            b_33 = 0.5 * x(4) * Aircraft_Kinematic.dt ;
            b_34 = -1 * 0.5 * x(5) * Aircraft_Kinematic.dt ;
            b_41 = 0;
            b_42 = -1 * 0.5 * x(6) * Aircraft_Kinematic.dt;
            b_43 = 0.5 * x(5) * Aircraft_Kinematic.dt ;
            b_44 = 0.5 * x(4) * Aircraft_Kinematic.dt;
            B = [b_11 b_12 b_13 b_14; b_21 b_22 b_23 b_24; b_31 b_32 b_33 b_34; b_41 b_42 b_43 b_44];
            % Jacobian calc for linear part
            qq = q_rot.double; % put it back in double form
            a_11 = (qq(1)^2+qq(2)^2-qq(3)^2-qq(4)^2) * Aircraft_Kinematic.dt ;
            a_12 = 0;
            a_13 = 0;
            a_14 = 0;
            a_21 = (2*qq(2)*qq(3)+qq(1)*qq(4))* Aircraft_Kinematic.dt;
            a_22 = 0;
            a_23 = 0;
            a_24 = 0;
            a_31 = (2*qq(2)*qq(4)-qq(1)*qq(3)) * Aircraft_Kinematic.dt; 
            a_32 = 0;
            a_33 = 0;
            a_34 = 0;
            A = [a_11 a_12 a_13 a_14;a_21 a_22 a_23 a_24;a_31 a_32 a_33 a_34];
            
            J = zeros(7,4);
            J(1:3,1:4) = A;
            J(4:7,1:4) = B;
            
        end
        function J = df_dw_func(x,u,w) % noise Jacobian
            g_11 = 0;
            g_12 = -1 * 0.5 * x(5) * (Aircraft_Kinematic.dt^0.5) ;
            g_13 = -1 * 0.5 * x(6) * (Aircraft_Kinematic.dt^0.5) ;
            g_14 = -1 * 0.5 * x(7) * (Aircraft_Kinematic.dt^0.5);
            g_21 = 0 ;
            g_22 = 0.5 * x(4) * (Aircraft_Kinematic.dt^0.5);
            g_23 = -1 * 0.5 * x(7) * (Aircraft_Kinematic.dt^0.5) ;
            g_24 = 0.5 * x(6) * (Aircraft_Kinematic.dt^0.5) ;
            g_31 = 0 ;
            g_32 = 0.5 * x(7) * (Aircraft_Kinematic.dt^0.5);
            g_33 = 0.5 * x(4) * (Aircraft_Kinematic.dt^0.5) ;
            g_34 = -1 * 0.5 * x(5) * (Aircraft_Kinematic.dt^0.5) ;
            g_41 = 0;
            g_42 = -1 * 0.5 * x(6) * (Aircraft_Kinematic.dt^0.5);
            g_43 = 0.5 * x(5) * (Aircraft_Kinematic.dt^0.5) ;
            g_44 = 0.5 * x(4) * (Aircraft_Kinematic.dt^0.5);
            G = [g_11 g_12 g_13 g_14; g_21 g_22 g_23 g_24; g_31 g_32 g_33 g_34; g_41 g_42 g_43 g_44];
            % Jacobian for linear part
            rot  = [x(4) , x(5) , x(6) , x(7)];% rotation state
            q_rot = Quaternion(rot);
            q_rot = unit(q_rot);% making a normalized quarternion
            qq = q_rot.double; % put it back in double form
            a_11 = (qq(1)^2+qq(2)^2-qq(3)^2-qq(4)^2) * (Aircraft_Kinematic.dt)^0.5 ;
            a_12 = 0;
            a_13 = 0;
            a_14 = 0;
            a_21 = (2*qq(2)*qq(3)+qq(1)*qq(4))* (Aircraft_Kinematic.dt)^0.5;
            a_22 = 0;
            a_23 = 0;
            a_24 = 0;
            a_31 = (2*qq(2)*qq(4)-qq(1)*qq(3)) * (Aircraft_Kinematic.dt)^0.5; 
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
        function nominal_traj = generate_open_loop_point2point_traj(start,goal) % generates open-loop trajectories between two start and goal states
             % The MATLAB RVC ToolBox Must be loaded first
            disp('Using RRT3D to connect points on two orbits');
            disp('Start point is:');start
            disp('Goal point is:');goal
            disp('Solving...');
            veh = Aircraft_Kinematic();
            rrt = RRT3D([], veh, 'start', start, 'range', 5,'npoints',100,'speed',2,'time', Aircraft_Kinematic.dt);
            rrt.plan('goal',goal)   ;          % create navigation tree
            nominal_traj.x = [];
            nominal_traj.u = [];
            nominal_traj = rrt.path(start, goal) ; % animate path from this start location
            
            %% Code for plotting
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
                nominal_traj.x = nomXs;
                nominal_traj.u = controls;
            end
            
            disp('Done with RRT...');
%             
%             x = start;
%             X = [];
%             X = [X,x];
%             w = [0,0,0,0]';
%             for i=1:length(controls(1,:))
%                 u = controls(:,i);
%                 x = Aircraft_Kinematic.f_discrete(x,u,w);
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
                orbit = Aircraft_Kinematic.generate_orbit(orbit_center);
                %orbit = Aircraft_Kinematic.draw_orbit(orbit);
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
            orbit_length_time_continuous = orbit_length_meter/Aircraft_Kinematic.linear_velocity_min_on_orbit;
            T_rational = orbit_length_time_continuous/Aircraft_Kinematic.dt;
            T = ceil(T_rational);
            orbit.period = T;
            orbit.center = orbit_center;
            
            % defining controls on the orbit
            V_p = Aircraft_Kinematic.linear_velocity_min_on_orbit * [ones(1,T-1) , T_rational-floor(T_rational)]; % we traverse the orbit with minimum linear velocity
            omega_p = Aircraft_Kinematic.angular_velocity_max * [ones(1,T-1) , T_rational-floor(T_rational)]; % we traverse the orbit with maximum angular velocity
            u_p = [V_p;zeros(1,T);zeros(1,T);omega_p];
            w_zero = Aircraft_Kinematic.zeroNoise; % no noise
            
            % defining state steps on the orbit
            zeroQuaternion = state.zeroQuaternion; % The robot's angle in the start of orbit is zero
            x_p(:,1) = [orbit_center.val(1:3) - [0;orbit.radius;0] ; zeroQuaternion']; % initial x
            for k=1:T
                x_p(:,k+1) = MotionModel_class.f_discrete(x_p(:,k),u_p(:,k),w_zero);
            end
            orbit.x = x_p(:,1:T);  % "x_p" is of length T+1, but "x_p(:,T+1)" is equal to "x_p(:,1)"
            orbit.u = u_p;  % "u_p" is of length T.
            orbit.plot_handle = [];
        end
        
        function YesNo = is_constraints_violated(open_loop_traj)
            error('not yet implemented');
        end
        function traj_plot_handle = draw_nominal_traj(nominal_traj, varargin)
            error('not yet implemented');
        end
    end
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