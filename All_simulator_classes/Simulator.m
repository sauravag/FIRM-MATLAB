 % This is a dummy class. It is ''typedefed'' from the oringinal class ''EmbeddedSimulator''.
% If you want to make any changes you need to change the original class.
classdef Simulator < SimulatorInterface
    properties
        sceneHierarchy %% an structure containing Scene Hierarchy (objects in the scene like floor , walls) and their children and properties
        robot
        par
        obstacle
        simulatorName = 'Embedded';
        belief
    end
    
    methods
        % constructor
        function obj = Simulator()
            % in constructor we retrive the paraemters of the planning
            % problem entered by the user.
            obj.par = user_data_class.par.sim;
            %             obj.robot = Robot([0;0;0]);
        end
        % initialize : initializes the simulator
        function obj = initialize(obj)
<<<<<<< Updated upstream
            old_prop = obj.set_figure(); %#ok<NASGU>
            % Following "if" statements cause the code to first construct the existing
            % parts of the environment, and then construct the parts that
            % the user is going to build.
            if user_data_class.par.observation_model_parameters.interactive_OM == 0
                OM = ObservationModel_class; % The object OM is only created for "Constant" properties of the "ObservationModel_class" class to be initialized.
                OM = OM.draw(); %#ok<NASGU>
=======
            % Transfer the .obj file (take the path from .obj file generator)
            %             [res(1)] = obj.vrep.simxTransferFile(obj.clientID,'/home/ajinkya/Dropbox/FIRM_toolbox_ver_current/cube1.obj','cube1.obj',20,obj.vrep.simx_opmode_oneshot_wait);
            %
            %           % Initializing scene saving
            %            [res(2),replyData] = obj.vrep.simxQuery(obj.clientID,'request','save','reply',5000);
            %            if (res==obj.vrep.simx_error_noerror)
            %                fprintf('scene saved\n');
            %                obj.scene = replyData;
            %                fprintf('scene saved at location: %s\n', obj.scene);
            %            end
            
            
            % Taking user input for choice of planner and robot model
            %             fprintf('Enter the name of the robot model used : \ndr12 or dr20 or youbot\n');
            %             obj.robotModel = input('Enter the model : ','s');
            %             obj.robotModel = 'dr20';
            %               obj.robotModel = 'dr12';
            obj.robotModel = 'youbot';
            %             fprintf('Enter choice for the planner\n for FIRM enter "1" \tor\tfor V-Rep internal Planner enter "0" :\n')
            %             obj.planner = str2num(input('Enter the planner : ','s'));
            disp('planner is FIRM \n')
            disp(['robot is :' ,obj.robotModel])
            obj.planner= 1;
            
            % Loading the pre-customized environments and obstacles based on user choice
            if(obj.planner==1)
                if(strcmp(obj.robotModel,'dr12'))
                    obj.scene = fullfile(fullfile(fileparts(which('Main.m')),'All_simulator_classes','v_rep_models'),'laser_test_dr12_with_control.ttt');
                elseif(strcmp(obj.robotModel,'dr20'))
                    obj.scene = fullfile(fullfile(fileparts(which('Main.m')),'All_simulator_classes','v_rep_models'),'laser_test_dr20_with_control.ttt');
                    
                elseif(strcmp(obj.robotModel,'youbot'))
                    %                     obj.scene = fullfile(fullfile(fileparts(which('Main.m')),'All_simulator_classes','v_rep_models'),'laser_test_youbot_with_control.ttt');%'C:\Users\Ajinkya\Documents\GitHub\FIRM-MATLAB\Aji_V-rep\laser_test_20.ttt';
                    obj.scene = fullfile(fullfile(fileparts(which('Main.m')),'All_simulator_classes','v_rep_models'),'youbot_test_w_laser.ttt');%'C:\Users\Ajinkya\Documents\GitHub\FIRM-MATLAB\Aji_V-rep\laser_test_20.ttt';
                    
                end
                
            elseif(obj.planner==0)
                if(strcmp(obj.robotModel,'dr12'))
                    obj.scene = fullfile(fullfile(fileparts(which('Main.m')),'All_simulator_classes','v_rep_models'),'laser_test_dr12.ttt');
                elseif(strcmp(obj.robotModel,'dr20'))
                    obj.scene = fullfile(fullfile(fileparts(which('Main.m')),'All_simulator_classes','v_rep_models'),'env_fourthfloor.ttt');
                elseif(strcmp(obj.robotModel,'youbot'))
                    %                     obj.scene = fullfile(fullfile(fileparts(which('Main.m')),'All_simulator_classes','v_rep_models'),'laser_test_youbot_with_control.ttt');%'C:\Users\Ajinkya\Documents\GitHub\FIRM-MATLAB\Aji_V-rep\laser_test_20.ttt';
                    obj.scene = fullfile(fullfile(fileparts(which('Main.m')),'All_simulator_classes','v_rep_models'),'youbot_test_w_laser.ttt');%'C:\Users\Ajinkya\Documents\GitHub\FIRM-MATLAB\Aji_V-rep\laser_test_20.ttt';
                    
                end
>>>>>>> Stashed changes
            end
            if obj.par.intractive_obst == 0
                obj.obstacle = obstacles_class; % The object "Obstacles" is never used. This line only cause the "Constant" properties of the "obstacles_class" class to be initialized.
                obj.obstacle = obj.obstacle.draw();
            end
            
<<<<<<< Updated upstream
            if obj.par.intractive_obst == 1
                obj.obstacle = obstacles_class; % The object "Obstacles" is never used. This line only cause the "Constant" properties of the "obstacles_class" class to be initialized.
            end
            if user_data_class.par.observation_model_parameters.interactive_OM == 1
                OM = ObservationModel_class; % The object OM is only created for "Constant" properties of the "ObservationModel_class" class to be initialized.
                OM.plot_handle = OM.draw();
=======
            % Initializing the robot
            
            % Loading the Robot
            %[res(7),obj.robot] = obj.vrep.simxLoadModel(obj.clientID,'/home/ajinkya/Dropbox/summer13/V-REP_PRO_EDU_V3_0_3_Linux/models/robots/mobile/dr20.ttm',0,obj.vrep.simx_opmode_oneshot_wait);
            
            
            if(strcmp(obj.robotModel,'dr12'))
                obj.interWheelDistance = 0.154;
                [res, obj.robot] = obj.vrep.simxGetObjectHandle(obj.clientID,'dr12',obj.vrep.simx_opmode_oneshot_wait);
                
            elseif(strcmp(obj.robotModel,'dr20'))
                obj.interWheelDistance = 0.254;
                [res, obj.robot] = obj.vrep.simxGetObjectHandle(obj.clientID,'dr20',obj.vrep.simx_opmode_oneshot_wait);
            elseif(strcmp(obj.robotModel,'youbot'))
                
                %                 [res, obj.robot] = obj.vrep.simxGetObjectHandle(obj.clientID,'youBot#0',obj.vrep.simx_opmode_oneshot_wait);
                [res, obj.robot] = obj.vrep.simxGetObjectHandle(obj.clientID,'youBot',obj.vrep.simx_opmode_oneshot_wait);
                %% obtaining the handles for the
                [res, obj.robot_joints.rollingJoint_fl] = obj.vrep.simxGetObjectHandle(obj.clientID,'rollingJoint_fl',obj.vrep.simx_opmode_oneshot_wait); %1-->1
                [res, obj.robot_joints.rollingJoint_fr] = obj.vrep.simxGetObjectHandle(obj.clientID,'rollingJoint_fr',obj.vrep.simx_opmode_oneshot_wait); %4-->2
                [res, obj.robot_joints.rollingJoint_rl] = obj.vrep.simxGetObjectHandle(obj.clientID,'rollingJoint_rl',obj.vrep.simx_opmode_oneshot_wait); %2-->3
                [res, obj.robot_joints.rollingJoint_rr] = obj.vrep.simxGetObjectHandle(obj.clientID,'rollingJoint_rr',obj.vrep.simx_opmode_oneshot_wait); %3-->4
                
            end
            
            
            %Handles for various parts of robot
            if ~(strcmp(obj.robotModel,'youbot'))
                [res(8), obj.robot_joints] = obj.vrep.simxGetObjects(obj.clientID, obj.vrep.sim_object_joint_type,obj.vrep.simx_opmode_oneshot_wait);
            end
            %Handle for the floor
            [res(21),obj.floor] = obj.vrep.simxGetObjectHandle(obj.clientID,'DefaultFloor',obj.vrep.simx_opmode_oneshot_wait);
            
            %Intializing the Environment
            [res(20)] = obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            pause(2);
            
            % checking for controlType
            if(obj.planner==1)
                %                 fprintf('Enter the control mode type\nEnter "1" for Kinematic or "2" for Dynamic\n');
                %                 mode = input('Enter Choice :');
                fprintf('mode "1" for Dynamic\n');
                mode = 1;
                
                if(mode==1)
                    obj.controlType = 'kinematic';
                    % making the robot dynamic setting
                    nLoopToBreak = 1;
                    while obj.vrep.simxSetModelProperty( obj.clientID, obj.robot, int32(0),obj.vrep.simx_opmode_oneshot_wait)~=obj.vrep.sim_script_no_error
                        nLoopToBreak =nLoopToBreak+1 ;
                        if nLoopToBreak>=10
                            break
                            disp('ERROR: could not make the robot dynamic')
                        end
                        
                    end
                    if nLoopToBreak<10
                        
                        [ errorCode]=obj.vrep.simxSetModelProperty( obj.clientID, obj.robot, int32(0),obj.vrep.simx_opmode_oneshot_wait)
                        %making sure that the joint velocities are zero at the
                        %beginning
                        obj.vrep.simxPauseCommunication(obj.clientID,1); %% pause the communication temporariliy
                        [res_fl] = obj.vrep.simxSetJointTargetVelocity(obj.clientID,  obj.robot_joints.rollingJoint_fl,0,obj.vrep.simx_opmode_oneshot);%1-->1
                        [res_fr] = obj.vrep.simxSetJointTargetVelocity(obj.clientID,  obj.robot_joints.rollingJoint_fr, 0,obj.vrep.simx_opmode_oneshot);%4-->2
                        [res_rl] = obj.vrep.simxSetJointTargetVelocity(obj.clientID,  obj.robot_joints.rollingJoint_rl,0,obj.vrep.simx_opmode_oneshot);%2-->3
                        [res_rr] = obj.vrep.simxSetJointTargetVelocity(obj.clientID,  obj.robot_joints.rollingJoint_rr, 0,obj.vrep.simx_opmode_oneshot);%3-->4
                        obj.vrep.simxPauseCommunication(obj.clientID,0); %% resume the communication
                        
                    end
                    nLoopToBreak = 1;
                    while obj.vrep.simxSetModelProperty( obj.clientID, obj.robot, obj.vrep.sim_modelproperty_not_dynamic,obj.vrep.simx_opmode_oneshot_wait)~=obj.vrep.sim_script_no_error
                        nLoopToBreak =nLoopToBreak+1 ;
                        if nLoopToBreak>=10
                            break
                            disp('ERROR: could not make the robot static')
                        end
                        
                    end
                    
                    
                    [res]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_fl,obj.vrep.simx_opmode_streaming);
                    [res]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_fr,obj.vrep.simx_opmode_streaming);
                    [res]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_rl,obj.vrep.simx_opmode_streaming);
                    [res]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_rr,obj.vrep.simx_opmode_streaming);
                    [res] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.robot,-1, obj.vrep.simx_opmode_streaming);
                    [res] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.robot, -1,obj.vrep.simx_opmode_streaming);
                    
                elseif(mode==2)
                    obj.controlType = 'dynamic';
                    %                     [res(14)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(2), 0,obj.vrep.simx_opmode_streaming);
                    %                     [res(15)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(3), 0,obj.vrep.simx_opmode_streaming);
                end
>>>>>>> Stashed changes
            end
            
            % video making
            if obj.par.video == 1;
                global vidObj; %#ok<TLEV>
                [file,path] = uiputfile('OnlinePhaseVideo.avi','Save the runtime video as');
                vidObj = VideoWriter(fullfile(path,file));
                vidObj.Quality = obj.par.video_quality;
                vidObj.FrameRate = obj.par.FrameRate;
                open(vidObj);
            end
            %obj = Environment_construction(obj); % Construct the environment (obstacles, landmarks, PRM)
            if ~strcmpi(obj.par.env_background_image_address,'none') % check to see if the environment has any background picuture or not
                background = imread(obj.par.env_background_image_address);
                smaller_background=imresize(background,obj.par.imageResizeRatio);
                smaller_background = flipdim(smaller_background,1);
                warp(smaller_background); axis on; set(gca,'Ydir','normal');view(0,90)
            end
            if obj.par.Lighting_and_3D_plots == 1
                view(obj.par.viewAngle);
                camlight('right')
                camzoom(obj.par.initialZoomRatio)
            end
            obj.robot = state();
            obj.belief = belief();
        end
<<<<<<< Updated upstream
        % SetRobot : change robot parameters
        function obj = setRobot(obj,robot)
            if ~isfield(obj.robot,'plot_handle') || isempty(obj.robot.plot_handle) % if this is empty, it shows that the robot field is not initialized yet or we have deleted
                % its handle that is we want to dreaw ir wirh a new handle
                obj.robot = robot;
            else
                % otherwose just update the value
                obj.robot.val = robot.val;
            end
        end
        % GetRobot : get robot parameters
        function robot = getRobot(obj)
            robot = obj.robot;
=======
        
        %% Setting up the Robot
        function obj = setRobot(obj,position)
            if isa(position,'state'), position = position.val; end
            
            %Setting position of the robot
            % remember to convert position[1] to position.val[1] and do so
            % when trying to use the full closed loop problem
            
            if(strcmp(obj.robotModel,'dr12'))
                [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[position(1),position(2), 0.0787],obj.vrep.simx_opmode_oneshot);
                [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,-(pi/2),position(3)],obj.vrep.simx_opmode_oneshot);
            elseif(strcmp(obj.robotModel,'dr20'))
                [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[position(1),position(2), 0.1517],obj.vrep.simx_opmode_oneshot);
                [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,0,position(3)],obj.vrep.simx_opmode_oneshot);
            elseif (strcmp(obj.robotModel,'youbot'))
                %                 [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[position(1),position(2), 0.0957],obj.vrep.simx_opmode_oneshot);
                %                 [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,obj.floor,[0,0,position(3)],obj.vrep.simx_opmode_oneshot);
%                 obj = obj.getRobot();
                %                 turn = (pi*position(3)/180) - obj.robot_orientation(3);
                %                 orientation = obj.robot_orientation(3);
                % making the object static
                %                 [ errorCode]=obj.vrep.simxSetModelProperty( obj.clientID, obj.robot, obj.vrep.sim_modelproperty_not_dynamic,obj.vrep.simx_opmode_oneshot_wait)
                
                [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[position(1),position(2), 0.0957],obj.vrep.simx_opmode_oneshot_wait);
                [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[-pi/2,position(3),-pi/2],obj.vrep.simx_opmode_oneshot_wait);
                % returning to dynamic setting
                %                 [ errorCode]=obj.vrep.simxSetModelProperty( obj.clientID, obj.robot, int32(0),obj.vrep.simx_opmode_oneshot_wait)
                
            end
        end
        
        %% Acquiring the Position of Data
        function obj = getRobot(obj)
            [res(11),obj.robot_position] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.robot,-1, obj.vrep.simx_opmode_buffer);
            [res(12),obj.robot_orientation] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.robot, -1,obj.vrep.simx_opmode_buffer);
            obj.robot_position = reshape(obj.robot_position,length(obj.robot_position),1); % making sure that the vector is in column format
            obj.robot_orientation = reshape(obj.robot_orientation,length(obj.robot_orientation),1); % making sure that the vector is in column format
>>>>>>> Stashed changes
        end
        % Refresh :
        function obj = refresh(obj)
            obj.robot = obj.robot.delete_plot();
            obj.robot = obj.robot.draw();
%             obj.belief = obj.belief.delete_plot();
%             obj.belief = obj.belief.draw();
        end
<<<<<<< Updated upstream
        function b = getBelief(obj)
            b=obj.belief;
        end
        function obj = setBelief(obj,b)
            
            if ~isfield(obj.belief,'ellipse_handle') || ~isempty(obj.belief.ellipse_handle) || ~isempty(obj.belief.est_mean.plot_handle)
                % if any of the belief object's graphics handles are
                % non-empty, we just
                obj.belief.est_cov = b.est_cov;
                obj.belief.est_mean.val = b.est_mean.val;
            else
                obj.belief = b;
=======
        
        %% Evolving the function
        function obj = evolve(obj,control)
            
            if(obj.planner==1)
                % Getting control signals from FIRM
                % Note: The units of the control signals must be the same
                % as chosen by the user in v-rep.The user settings in the
                % v-rep can be adjusted by going to Vrep window, Menu
                % bar -> Tools -> User Settings . Currently, the user
                % ssettings should be set to length in 'meter', angle in
                % 'radians' adn time in 'seconds'. So, velocity is in 'm/s'
                % and angular velocity is in 'rad/s'
                
                linear_velocity = control(1);
                ang_velocity = control(2);
                
                if(strcmp(obj.robotModel,'dr12'))
                    wheelDiameter = 0.086;
                    % Converting control signals to wheel velocities
                    obj.leftJointVelocity = (linear_velocity - ((obj.interWheelDistance*ang_velocity)/2))*(2/wheelDiameter);
                    obj.rightJointVelocity = (linear_velocity + ((obj.interWheelDistance*ang_velocity)/2))*(2/wheelDiameter);
                    
                    if(strcmp(obj.controlType,'dynamic'))
                        % Since simulation is dynamic we have to set the
                        % joint (motor) velocities. Since Vrep tends to
                        % generate realistic dynamic siulations, so one cannot
                        % set very high linear and angular velocities as they will lead to more
                        % errorneous results. Also, motors will take
                        % some finite time to reach the setted target
                        % velocity and jumps won't be discrete
                        
                        [res(14)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(2), obj.leftJointVelocity,obj.vrep.simx_opmode_oneshot);
                        [res(15)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(3), obj.rightJointVelocity,obj.vrep.simx_opmode_oneshot);
                        
                        
                    elseif(strcmp(obj.controlType,'kinematic'))
                        
                        % Getting the joint positions
                        
                        [res(14),p]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(2),obj.vrep.simx_opmode_buffer);
                        [res(14),q]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(3),obj.vrep.simx_opmode_buffer);
                        
                        linMov = obj.dt*linear_velocity; % linear displacement
                        rotMov = obj.dt*ang_velocity; % Angular dispalcement
                        obj = obj.getRobot(); % Acquiring position
                        xDir = [cos(obj.robot_orientation(3)),sin(obj.robot_orientation(3)),0.0];
                        obj.robot_position(1) = obj.robot_position(1) + xDir(1)*linMov;
                        obj.robot_position(2) = obj.robot_position(2) + xDir(2)*linMov;
                        obj.robot_orientation(3) = obj.robot_orientation(3) + rotMov;
                        
                        % Updating the joint position with time
                        [res(16)] = obj.vrep.simxPauseCommunication(obj.clientID,1);
                        % Pausing the communication will ensure
                        % simulataneous application of the following
                        % commands.
                        
                        % Updating Joints' position
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(2),(p+((obj.leftJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(3),(q+((obj.rightJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        
                        % Setting robot's position
                        [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[obj.robot_position(1),obj.robot_position(2), 0.0787],obj.vrep.simx_opmode_oneshot);
                        [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,-(pi/2),obj.robot_orientation(3)],obj.vrep.simx_opmode_oneshot);
                        [res(16)] = obj.vrep.simxPauseCommunication(obj.clientID,0); % Resuming communication
                    end
                    
                elseif(strcmp(obj.robotModel,'dr20'))
                    wheelDiameter = 0.085;
                    % Converting control signals to wheel velocities
                    obj.leftJointVelocity = (linear_velocity - ((obj.interWheelDistance*ang_velocity)/2))*(2/wheelDiameter);
                    obj.rightJointVelocity = (linear_velocity + ((obj.interWheelDistance*ang_velocity)/2))*(2/wheelDiameter);
                    
                    if(strcmp(obj.controlType,'dynamic'))
                        [res(14)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(2), obj.leftJointVelocity,obj.vrep.simx_opmode_oneshot);
                        [res(15)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(3), obj.rightJointVelocity,obj.vrep.simx_opmode_oneshot);
                        
                    elseif(strcmp(obj.controlType,'kinematic'))
                        
                        % Getting the joint positions
                        [res(14),p]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(2),obj.vrep.simx_opmode_streaming);
                        [res(14),q]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(3),obj.vrep.simx_opmode_streaming);
                        
                        [res(14),p]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(2),obj.vrep.simx_opmode_buffer);
                        [res(14),q]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(3),obj.vrep.simx_opmode_buffer);
                        
                        linMov = obj.dt*linear_velocity;
                        rotMov = obj.dt*ang_velocity;
                        obj = obj.getRobot();
                        xDir = [cos(obj.robot_orientation(3)),sin(obj.robot_orientation(3)),0.0];
                        obj.robot_position(1) = obj.robot_position(1) + xDir(1)*linMov;
                        obj.robot_position(2) = obj.robot_position(2) + xDir(2)*linMov;
                        obj.robot_orientation(3) = obj.robot_orientation(3) + rotMov;
                        
                        % Updating the joint position with time
                        [res(16)] = obj.vrep.simxPauseCommunication(obj.clientID,1);
                        % Pausing the communication will ensure
                        % simulataneous application of the following
                        % commands.
                        
                        % Updating Joints' position
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(2),(p+((obj.leftJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(3),(q+((obj.rightJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        
                        % Setting robot's position
                        [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[obj.robot_position(1),obj.robot_position(2), 0.1517],obj.vrep.simx_opmode_oneshot);
                        [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,0,obj.robot_orientation(3)],obj.vrep.simx_opmode_oneshot);
                        [res(16)] = obj.vrep.simxPauseCommunication(obj.clientID,0);% Resuming communication between MATLAB and V-rep
                    end
                    
                elseif(strcmp(obj.robotModel,'youbot'))
                    wheelDiameter = 0.085;
                    % Converting control signals to wheel velocities
                    
                    if(strcmp(obj.controlType,'dynamic'))
                        
                        
                        %% Set the joint velocities
                        vel_w_fl = control(1);
                        vel_w_fr = control(2);
                        vel_w_rl = control(3);
                        vel_w_rr = control(4);
                        obj.vrep.simxPauseCommunication(obj.clientID,1); %% pause the communication temporariliy
                        [res_fl] = obj.vrep.simxSetJointTargetVelocity(obj.clientID,  obj.robot_joints.rollingJoint_fl,vel_w_fl,obj.vrep.simx_opmode_oneshot_wait);%1-->1
                        [res_fr] = obj.vrep.simxSetJointTargetVelocity(obj.clientID,  obj.robot_joints.rollingJoint_fr, vel_w_fr,obj.vrep.simx_opmode_oneshot_wait);%4-->2
                        [res_rl] = obj.vrep.simxSetJointTargetVelocity(obj.clientID,  obj.robot_joints.rollingJoint_rl,vel_w_rl,obj.vrep.simx_opmode_oneshot_wait);%2-->3
                        [res_rr] = obj.vrep.simxSetJointTargetVelocity(obj.clientID,  obj.robot_joints.rollingJoint_rr, vel_w_rr,obj.vrep.simx_opmode_oneshot_wait);%3-->4
                        obj.vrep.simxPauseCommunication(obj.clientID,1); %% resume the communication
                        
                    elseif(strcmp(obj.controlType,'kinematic'))
                        
                        % Getting the joint positions
                        
                        [res(14),rollingJoint_fl]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_fl,obj.vrep.simx_opmode_buffer);
                        [res(14),rollingJoint_fr]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_fr,obj.vrep.simx_opmode_buffer);
                        [res(14),rollingJoint_rl]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_rl,obj.vrep.simx_opmode_buffer);
                        [res(14),rollingJoint_rr]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_rr,obj.vrep.simx_opmode_buffer);
                        currentPosition = [obj.getRobot().robot_position(1);obj.getRobot().robot_position(2);obj.getRobot().robot_orientation(2)];
                        newPosition = MotionModel_class.f_discrete(currentPosition,control,zeros(MotionModel_class.wDim,1));
                        
                        
                        obj.robot_position(1) = newPosition(1);
                        obj.robot_position(2) = newPosition(2);
                        obj.robot_orientation(3) = newPosition(3);
                        
                        
                                               % Setting robot's position
                        [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[obj.robot_position(1),obj.robot_position(2),  0.0957],obj.vrep.simx_opmode_oneshot_wait);
                        [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[-pi/2,obj.robot_orientation(3),-pi/2],obj.vrep.simx_opmode_oneshot_wait);
      
                        
                        
                        % Updating the joint position with time
                        [res(16)] = obj.vrep.simxPauseCommunication(obj.clientID,1);
                        % Pausing the communication will ensure
                        % simulataneous application of the following
                        % commands.
                        
%                         % Setting robot's position
%                         [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[obj.robot_position(1),obj.robot_position(2),  0.0957],obj.vrep.simx_opmode_oneshot);
%                         [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[-pi/2,obj.robot_orientation(3),-pi/2],obj.vrep.simx_opmode_oneshot);
%                         
                        
                        % Updating Joints' position
                        vel_w_fl = control(1);
                        vel_w_fr = control(2);
                        vel_w_rl = control(3);
                        vel_w_rr = control(4);
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_fl,(rollingJoint_fl+((vel_w_fl*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_fr,(rollingJoint_fr+((vel_w_fr*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_rl,(rollingJoint_rl+((vel_w_rl*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_rr,(rollingJoint_rr+((vel_w_rr*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        
                        
                        [res(16)] = obj.vrep.simxPauseCommunication(obj.clientID,0);% Resuming communication between MATLAB and V-rep
                    end
                    
                    
                    
                    
                end
>>>>>>> Stashed changes
            end
        end
        % stopRun (not sure about this one)
        function obj = simStop(obj)
            disp('Simulation Stopped')
        end
        % evolve : evolve robot
        function obj = evolve(obj,u,varargin)
            if nargin==3
                noiseMode = varargin{1};
            else
                noiseMode = 1; % by default we add noise 
            end
            if noiseMode
                w = MotionModel_class.generate_process_noise(obj.robot.val,u);
            else
                w = MotionModel_class.zeroNoise;
            end
            obj.robot.val = MotionModel_class.f_discrete(obj.robot.val,u,w);
        end
        
        function z = getObservation(obj, noiseMode)
            % generating observation noise
            if noiseMode
                v = ObservationModel_class.generate_observation_noise(obj.robot.val);
            else
                v = ObservationModel_class.zeroNoise;
            end
            % constructing ground truth observation
            z = ObservationModel_class.h_func(obj.robot.val,v);
        end
        
        
    end
    
    methods (Access = private)
        function old_prop = set_figure(obj) % This function sets the figure (size and other properties) to values that are needed for landmark selection or drawing.
            figure(gcf);
            if ~strcmpi(obj.par.env_background_image_address,'none') % check to see if the environment has any background picuture or not
                background = imread(obj.par.env_background_image_address);
                smaller_background=imresize(background,obj.par.imageResizeRatio);
                smaller_background = flipdim(smaller_background,1);
                imshow(smaller_background); axis on; set(gca,'Ydir','normal');view(0,90)
            end
            old_prop{1}=get(gca,'NextPlot');hold on; % save the old "NextPlot" property and set it to "hold on" % Note that this procedure cannot be moved into the "set_figure" function.
            old_prop{2}=get(gca,'XGrid'); % save the old "XGrid" property.
            old_prop{3}=get(gca,'YGrid'); % save the old "YGrid" property.
            grid on; % set the XGrid and YGrid to "on".
            if ~isempty(obj.par.figure_position)
                set(gcf,'Position',obj.par.figure_position)
            end
            axis(obj.par.env_limits);
            set(gca,'DataAspectRatio',[1 1 1]); % makes the scaling of different axes the same. So, a circle is shown as a circle not ellipse.
            %             gray_box_position = get(gca, 'OuterPosition');
            %             set(gca, 'Position', gray_box_position);
        end
    end
    
end
 % This is a dummy class. It is ''typedefed'' from the oringinal class ''EmbeddedSimulator''.
% If you want to make any changes you need to change the original class.
