%%%%%%%%%%%%%%%%%%%% Vrep Simulator Interface Class %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author  :   Ajinkya Jain
%   email   :   jainajinkya92@gmail.com
%   Date    :   July 2013
%   Place   :   Dept. of Aerospace Engg., Texas A&M University, College
%               Station, TX, US
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Developed as a part of FIRM Toolbox for Matlab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef VRepSimulator < SimulatorInterface
    properties
        %connection Property
        vrep;
        clientID;
        connectionStatus=0;
        simulatorName = 'vrep';
        
        % Simulation Property
        newScene;
        dt = 0.05;
        controlType;
        planner;
        
        %Scene Properties
        scene;
        floor;
        obstacles;
        numberOfObjects=1;
        
        % Robot Properties
        robotModel;
        robot;
        robot_joints;
        robot_position;
        robot_orientation;
        robot_body;
        interWheelDistance;
        leftJointVelocity;
        rightJointVelocity;
        
        % Sensor properties
        sensorChosen;
        sensorID;
        sensor;
    end
    
    methods
        %% Constructor
        function obj = VRepSimulator()
            % checking for correct dll file for connection
            is32=exist([matlabroot,'/bin/win32'],'dir')~=0;
            is64=exist([matlabroot,'/bin/win64'],'dir')~=0;
            if is32
                obj.vrep = remApi('remoteApi_32', 'extApi.h');
                disp('test')
            elseif is64
                obj.vrep = remApi('remoteApi_64', 'extApi.h');
                disp('test')
                
            end
            
            % Setting up the connection
            while(obj.connectionStatus==0) %Checking for the connection establishment
                obj.clientID = obj.vrep.simxStart('127.0.0.1',19999,true,true,5000,5); % port should be taken as input. options number < 20000
                if(obj.clientID>-1)
                    fprintf('Connecton Established\n');
                    obj.connectionStatus = 1;
                else fprintf('Connection Failed...\nRetrying\n');
                end
            end
        end
        
        %% Destructor
        function obj = simDelete(obj)
            
            fprintf('Stopping Simulation and Calling Destructor\n');
            [res] = obj.vrep.simxStopSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot_wait); % Stopping Simulation
            [res] = obj.vrep.simxCloseScene(obj.clientID, obj.vrep.simx_opmode_oneshot_wait); % Closing the loaded V-Rep scene
            obj.vrep.simxFinish(obj.clientID);  % Ending the connection
            obj.vrep.delete();
            fprintf('Simulation Ended\n');
        end
        
        
        %% Setting up the environment
        function obj = initialize(obj)
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
            end
            
            [res] = obj.vrep.simxLoadScene(obj.clientID,obj.scene,0,obj.vrep.simx_opmode_oneshot_wait); % Loading the scene
            
            % Getting handlers of all the obstacles present in the
            % environment
            for i=1:obj.numberOfObjects
                [res, obj.obstacles(i)] = obj.vrep.simxGetObjectHandle(obj.clientID,num2str(i),obj.vrep.simx_opmode_oneshot_wait);
                
                % changing the parameteres of the scene objects
                [res] = obj.vrep.simxSetObjectIntParameter(obj.clientID, obj.obstacles(i), 3003, 0,obj.vrep.simx_opmode_oneshot_wait);
                [res] = obj.vrep.simxSetObjectIntParameter(obj.clientID, obj.obstacles(i), 3004, ~0,obj.vrep.simx_opmode_oneshot_wait);
            end
            
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
                [res, obj.robot_joints] = obj.vrep.simxGetObjects(obj.clientID, obj.vrep.sim_object_joint_type,obj.vrep.simx_opmode_oneshot_wait);
            end
            %Handle for the floor
            [res,obj.floor] = obj.vrep.simxGetObjectHandle(obj.clientID,'DefaultFloor',obj.vrep.simx_opmode_oneshot_wait);
            
            %Intializing the Environment
            [res] = obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
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
            end
            
            % Initialzing Sensors
            obj.sensorChosen = 'laser';
            switch obj.sensorChosen
                case 'laser'
                    obj.sensorID = 1;
                    obj.sensor = laserScanner(obj.vrep,obj.clientID,obj.robot,obj.controlType);
                    % can add more cases for other types of sensors
            end
            
            pause(2); % Giving time to V-Rep to initialize all the settings we modified
            
            
            % Triggering on the synchronous mode
            [res] = obj.vrep.simxSynchronous(obj.clientID,1);
            
        end
        
        %% Setting up the Robot
        function obj = setRobot(obj,position)
            if isa(position,'state'), position = position.val; end
            
            %Setting position of the robot
            % remember to convert position[1] to position.val[1] and do so
            % when trying to use the full closed loop problem
            
            if(strcmp(obj.robotModel,'dr12'))
                [res] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[position(1),position(2), 0.0787],obj.vrep.simx_opmode_oneshot);
                [res] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,-(pi/2),position(3)],obj.vrep.simx_opmode_oneshot);
            elseif(strcmp(obj.robotModel,'dr20'))
                [res] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[position(1),position(2), 0.1517],obj.vrep.simx_opmode_oneshot);
                [res] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,0,position(3)],obj.vrep.simx_opmode_oneshot);
            elseif (strcmp(obj.robotModel,'youbot'))
                %                 [res] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[position(1),position(2), 0.0957],obj.vrep.simx_opmode_oneshot);
                %                 [res] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,obj.floor,[0,0,position(3)],obj.vrep.simx_opmode_oneshot);
%                 obj = obj.getRobot();
                %                 turn = (pi*position(3)/180) - obj.robot_orientation(3);
                %                 orientation = obj.robot_orientation(3);
                % making the object static
                %                 [ errorCode]=obj.vrep.simxSetModelProperty( obj.clientID, obj.robot, obj.vrep.sim_modelproperty_not_dynamic,obj.vrep.simx_opmode_oneshot_wait)
                
                [res] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[position(1),position(2), 0.0957],obj.vrep.simx_opmode_oneshot_wait);
                [res] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[-pi/2,position(3),-pi/2],obj.vrep.simx_opmode_oneshot_wait);
                % returning to dynamic setting
                %                 [ errorCode]=obj.vrep.simxSetModelProperty( obj.clientID, obj.robot, int32(0),obj.vrep.simx_opmode_oneshot_wait)
                
            end
        end
        
        %% Acquiring the Position of Data
        function obj = getRobot(obj)
            [res,obj.robot_position] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.robot,-1, obj.vrep.simx_opmode_buffer);
            [res,obj.robot_orientation] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.robot, -1,obj.vrep.simx_opmode_buffer);
            obj.robot_position = reshape(obj.robot_position,length(obj.robot_position),1); % making sure that the vector is in column format
            obj.robot_orientation = reshape(obj.robot_orientation,length(obj.robot_orientation),1); % making sure that the vector is in column format
        end
        
        %% Refreshing the Scene but we don't need to do this for V-Rep
        function obj = refresh(obj)
        end
        
        %% Evolving the function
        function obj = evolve(obj,control)
            
            %Triggering Simulation
%             [res] = obj.vrep.simxSynchronousTrigger(obj.clientID);
            
            
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
                        
                        [res] = obj.vrep.simxPauseCommunication(obj.clientID,1); % Pausing the communication
                        [res] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(2), obj.leftJointVelocity,obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(3), obj.rightJointVelocity,obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxPauseCommunication(obj.clientID,0); % Resuming the communication
                        
                    elseif(strcmp(obj.controlType,'kinematic'))
                        
                        % Getting the joint positions
                        
                        [res,p]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(2),obj.vrep.simx_opmode_buffer);
                        [res,q]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(3),obj.vrep.simx_opmode_buffer);
                        
                        linMov = obj.dt*linear_velocity; % linear displacement
                        rotMov = obj.dt*ang_velocity; % Angular dispalcement
                        obj = obj.getRobot(); % Acquiring position
                        xDir = [cos(obj.robot_orientation(3)),sin(obj.robot_orientation(3)),0.0];
                        obj.robot_position(1) = obj.robot_position(1) + xDir(1)*linMov;
                        obj.robot_position(2) = obj.robot_position(2) + xDir(2)*linMov;
                        obj.robot_orientation(3) = obj.robot_orientation(3) + rotMov;
                        
                        % Updating the joint position with time
                        [res] = obj.vrep.simxPauseCommunication(obj.clientID,1);
                        % Pausing the communication will ensure
                        % simulataneous application of the following
                        % commands.
                        
                        % Updating Joints' position
                        [res] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(2),(p+((obj.leftJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(3),(q+((obj.rightJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        
                        % Setting robot's position
                        [res] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[obj.robot_position(1),obj.robot_position(2), 0.0787],obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,-(pi/2),obj.robot_orientation(3)],obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxPauseCommunication(obj.clientID,0); % Resuming communication
                    end
                    
                elseif(strcmp(obj.robotModel,'dr20'))
                    wheelDiameter = 0.085;
                    % Converting control signals to wheel velocities
                    obj.leftJointVelocity = (linear_velocity - ((obj.interWheelDistance*ang_velocity)/2))*(2/wheelDiameter);
                    obj.rightJointVelocity = (linear_velocity + ((obj.interWheelDistance*ang_velocity)/2))*(2/wheelDiameter);
                    
                    if(strcmp(obj.controlType,'dynamic'))
                        [res] = obj.vrep.simxPauseCommunication(obj.clientID,1);
                        [res] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(2), obj.leftJointVelocity,obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(3), obj.rightJointVelocity,obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxPauseCommunication(obj.clientID,0);
                        
                    elseif(strcmp(obj.controlType,'kinematic'))
                        
                        % Getting the joint positions
                        [res,p]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(2),obj.vrep.simx_opmode_streaming);
                        [res,q]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(3),obj.vrep.simx_opmode_streaming);
                        
                        [res,p]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(2),obj.vrep.simx_opmode_buffer);
                        [res,q]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints(3),obj.vrep.simx_opmode_buffer);
                        
                        linMov = obj.dt*linear_velocity;
                        rotMov = obj.dt*ang_velocity;
                        obj = obj.getRobot();
                        xDir = [cos(obj.robot_orientation(3)),sin(obj.robot_orientation(3)),0.0];
                        obj.robot_position(1) = obj.robot_position(1) + xDir(1)*linMov;
                        obj.robot_position(2) = obj.robot_position(2) + xDir(2)*linMov;
                        obj.robot_orientation(3) = obj.robot_orientation(3) + rotMov;
                        
                        % Updating the joint position with time
                        [res] = obj.vrep.simxPauseCommunication(obj.clientID,1);
                        % Pausing the communication will ensure
                        % simulataneous application of the following
                        % commands.
                        
                        % Updating Joints' position
                        [res] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(2),(p+((obj.leftJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(3),(q+((obj.rightJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        
                        % Setting robot's position
                        [res] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[obj.robot_position(1),obj.robot_position(2), 0.1517],obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,0,obj.robot_orientation(3)],obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxPauseCommunication(obj.clientID,0);% Resuming communication between MATLAB and V-rep
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
                        obj.vrep.simxPauseCommunication(obj.clientID,0); %% resume the communication
                        
                    elseif(strcmp(obj.controlType,'kinematic'))
                        
                        % Getting the joint positions
                        
                        [res,rollingJoint_fl]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_fl,obj.vrep.simx_opmode_buffer);
                        [res,rollingJoint_fr]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_fr,obj.vrep.simx_opmode_buffer);
                        [res,rollingJoint_rl]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_rl,obj.vrep.simx_opmode_buffer);
                        [res,rollingJoint_rr]  = obj.vrep.simxGetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_rr,obj.vrep.simx_opmode_buffer);
                        currentPosition = [obj.getRobot().robot_position(1);obj.getRobot().robot_position(2);obj.getRobot().robot_orientation(2)];
                        newPosition = MotionModel_class.f_discrete(currentPosition,control,zeros(MotionModel_class.wDim,1));
                        
                        
                        obj.robot_position(1) = newPosition(1);
                        obj.robot_position(2) = newPosition(2);
                        obj.robot_orientation(3) = newPosition(3);
                        
                        
                        % Setting robot's position
                        [res] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[obj.robot_position(1),obj.robot_position(2),  0.0952],obj.vrep.simx_opmode_oneshot_wait);
                        [res] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[-pi/2,obj.robot_orientation(3),-pi/2],obj.vrep.simx_opmode_oneshot_wait);
      
                        
                        
                        % Updating the joint position with time
                        [res] = obj.vrep.simxPauseCommunication(obj.clientID,1);
                        % Pausing the communication will ensure
                        % simulataneous application of the following
                        % commands.
                        
%                         % Setting robot's position
%                         [res] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[obj.robot_position(1),obj.robot_position(2),  0.0957],obj.vrep.simx_opmode_oneshot);
%                         [res] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[-pi/2,obj.robot_orientation(3),-pi/2],obj.vrep.simx_opmode_oneshot);
%                         
                        
                        % Updating Joints' position
                        vel_w_fl = control(1);
                        vel_w_fr = control(2);
                        vel_w_rl = control(3);
                        vel_w_rr = control(4);
                        [res] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_fl,(rollingJoint_fl+((vel_w_fl*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_fr,(rollingJoint_fr+((vel_w_fr*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_rl,(rollingJoint_rl+((vel_w_rl*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints.rollingJoint_rr,(rollingJoint_rr+((vel_w_rr*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        
                        
                        [res] = obj.vrep.simxPauseCommunication(obj.clientID,0);% Resuming communication between MATLAB and V-rep
                    end
                    
                    
                    
                    
                end
            end
        end
        
        %% Sensor Data
        function obj = getSensorData(obj)
            switch obj.sensorID
                
                case 1
                    % Getting laser data
                    obj.sensor = obj.sensor.Scan(obj.vrep,obj.clientID,obj.robot,obj.controlType);
                    
                    % Add more cases for other sensors
            end
            
        end
        
        %% Pause Simulation
        function obj = simPause(obj)
            [res] = obj.vrep.simxPauseSimulation(obj.clientID,obj.vrep.simx_opmode_oneshot);
            fprintf('Simulation Paused\n');
        end
        
        %% Resume Simulation
        function obj = simResume(obj)
            [res] = obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            frpintf('Simulation resumed');
        end
        
        %% Stop Simulation
        function obj = simStop(obj)
            [res] = obj.vrep.simxStopSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot_wait);
            fprintf('Simulation Stopped\n');
        end
    end
end