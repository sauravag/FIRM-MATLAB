%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Vrep Simulator Interface Class%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author  :   Ajinkya Jain
%   email   :   jainajinkya92@gmail.com
%   Date    :   July 2013
%   Place   :   Dept. of Aerospace Engg., Texas A&M University, College
%               Station, TX, US
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Developed as a part of FIRM Toolbox for Matlab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef vrep_interface %< SimulatorInterface
    properties
        %connection Property
        vrep;
        clientID;
        % Simulation Property
        loop;
        dt = 0.05;
        controlType;
        controlMode;
        
        %Scene Properties
        scene;
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
        function obj = vrep_interface()
            is32=exist([matlabroot,'/bin/win32'],'dir')~=0;
            is64=exist([matlabroot,'/bin/win64'],'dir')~=0;
            if is32
                obj.vrep = remApi('remoteApi_32', 'extApi.h');
            elseif is64
                obj.vrep = remApi('remoteApi_64', 'extApi.h');
            end
            
            
            % Setting up the connection
            obj.clientID = obj.vrep.simxStart('127.0.0.1',19999,true,true,5000,5); % port should be taken as input. options number < 20000
            
            if(obj.clientID>-1)
                fprintf('Connecton Established\n');
            else fprintf('Connection Failed...\nRetry\n');
                obj.vrep.delete();
            end
        end
        
        %% Destructor
        function obj = delete(obj)
            
            fprintf('Stopping Simulation\n');
            [res(19)] = obj.vrep.simxStopSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot_wait);
            [res(18)] = obj.vrep.simxCloseScene(obj.clientID, obj.vrep.simx_opmode_oneshot_wait);
            obj.vrep.simxFinish(obj.clientID);
            obj.vrep.delete();
        end
    end
    
    methods
        
        %% Setting up the environment
        function obj = simInitialize(obj,controlMode)
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
            
            fprintf('Enter the name of the robot model used : \ndr12 or dr20\n');
            obj.robotModel = input('Enter the model : ','s');
            
            obj.controlMode = controlMode;
            % Loading the environment and obstacles
            if(obj.controlMode==1)
                if(strcmp(obj.robotModel,'dr12'))
                    obj.scene = fullfile(pwd,'laser_test_dr12_with_control.ttt');%'C:\Users\Ajinkya\Documents\GitHub\FIRM-MATLAB\Aji_V-rep\laser_test_20.ttt';
                elseif(strcmp(obj.robotModel,'dr20'))
                    obj.scene = fullfile(pwd,'laser_test_dr20_with_control.ttt');%'C:\Users\Ajinkya\Documents\GitHub\FIRM-MATLAB\Aji_V-rep\laser_test_20.ttt';
                end
                
            elseif(obj.controlMode==0)
                if(strcmp(obj.robotModel,'dr12'))
                    obj.scene = fullfile(pwd,'laser_test_dr12.ttt');%'C:\Users\Ajinkya\Documents\GitHub\FIRM-MATLAB\Aji_V-rep\laser_test_20.ttt';
                elseif(strcmp(obj.robotModel,'dr20'))
                    obj.scene = fullfile(pwd,'laser_test_dr20.ttt');%'C:\Users\Ajinkya\Documents\GitHub\FIRM-MATLAB\Aji_V-rep\laser_test_20.ttt';
                end
            end
            
            [res(3)] = obj.vrep.simxLoadScene(obj.clientID,obj.scene,0,obj.vrep.simx_opmode_oneshot_wait);
            
            for i=1:obj.numberOfObjects
                [res(4), obj.obstacles(i)] = obj.vrep.simxGetObjectHandle(obj.clientID,num2str(i),obj.vrep.simx_opmode_oneshot_wait);
                
                % changing the parameteres of the objects
                [res(5)] = obj.vrep.simxSetObjectIntParameter(obj.clientID, obj.obstacles(i), 3003, 0,obj.vrep.simx_opmode_oneshot_wait);
                [res(6)] = obj.vrep.simxSetObjectIntParameter(obj.clientID, obj.obstacles(i), 3004, ~0,obj.vrep.simx_opmode_oneshot_wait);
            end
        end
        
        %% Setting up the Robot
        function obj = SetRobot(obj,position,mode)
            %Loading the Robot
            %             [res(7),obj.robot] = obj.vrep.simxLoadModel(obj.clientID,'/home/ajinkya/Dropbox/summer13/V-REP_PRO_EDU_V3_0_3_Linux/models/robots/mobile/dr20.ttm',0,obj.vrep.simx_opmode_oneshot_wait);
            %
            %             % Dimensions of Robot
            %             [res(17),obj.robot_body] = obj.vrep.simxGetObjectHandle(obj.clientID, 'dr20_body_',obj.vrep.simx_opmode_oneshot_wait);
            %             [res(18),y_min] = obj.vrep.simxGetObjectFloatParameter(obj.clientID, obj.robot_body, 16, obj.vrep.simx_opmode_oneshot_wait);
            %             [res(19),y_max] = obj.vrep.simxGetObjectFloatParameter(obj.clientID, obj.robot_body, 19, obj.vrep.simx_opmode_oneshot_wait);
            %
            %             obj.interWheelDistance = y_max - y_min;
            
            if(strcmp(obj.robotModel,'dr12'))
                obj.interWheelDistance = 0.154;
                [res, obj.robot] = obj.vrep.simxGetObjectHandle(obj.clientID,'dr12',obj.vrep.simx_opmode_oneshot_wait);
                
            elseif(strcmp(obj.robotModel,'dr20'))
                obj.interWheelDistance = 0.254;
                [res, obj.robot] = obj.vrep.simxGetObjectHandle(obj.clientID,'dr20',obj.vrep.simx_opmode_oneshot_wait);
                
            end
            
            
            %Handles for various parts of robot
            [res(8), obj.robot_joints] = obj.vrep.simxGetObjects(obj.clientID, obj.vrep.sim_object_joint_type,obj.vrep.simx_opmode_oneshot_wait);
            
            %Setting initial position
            % remember to convert position[1] to position.val[1] and do so
            % when trying to use the full closed loop problem
            
            if(strcmp(obj.robotModel,'dr12'))
                [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[position(1),position(2), 0.0787],obj.vrep.simx_opmode_oneshot);
                [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,-(pi/2),position(3)],obj.vrep.simx_opmode_oneshot);
            elseif(strcmp(obj.robotModel,'dr20'))
                [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[position(1),position(2), 0.1517],obj.vrep.simx_opmode_oneshot);
                [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,0,position(3)],obj.vrep.simx_opmode_oneshot);
            end
            
            
            %Intializing the Environment
            [res(20)] = obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            [res(14)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(2), 0,obj.vrep.simx_opmode_streaming);
            [res(15)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(3), 0,obj.vrep.simx_opmode_streaming);
            %             [res(21)] = obj.vrep.simxPauseSimulation(obj.clientID,obj.vrep.simx_opmode_oneshot);
            obj.loop = 1;
            if(mode==1)
                obj.controlType = 'kinematic';
            elseif(mode==2)
                obj.controlType = 'dynamic';
            end
            pause(3);
            
            % Initialzing Sensors
            obj.sensorChosen = 'laser';
            switch obj.sensorChosen
                case 'laser'
                    obj.sensorID = 1;
                    obj.sensor = laserScanner(obj.vrep,obj.clientID,obj.robot,obj.controlType);
                    %                     obj.sensor = laser_Scanner(obj.vrep,obj.clientID,obj.robot);
                    
            end
            
            
        end
        
        %% Acquiring the Position of Data
        function obj = getRobot(obj)
            [res(11),obj.robot_position] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.robot,-1, obj.vrep.simx_opmode_oneshot_wait);
            [res(12),obj.robot_orientation] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.robot, -1,obj.vrep.simx_opmode_oneshot_wait);
        end
        
        %% Refreshing the Scene but we don't need to do this for V-Rep
        function obj = refresh(obj)
        end
        
        %% Evolving the function
        function obj = evolve(obj,control)
            
            if(obj.controlMode==1)
                linear_velocity = control(1);
                ang_velocity = control(2);
                
                if(strcmp(obj.robotModel,'dr12'))
                    wheelDiameter = 0.086;
                    obj.leftJointVelocity = (linear_velocity - ((obj.interWheelDistance*ang_velocity)/2))*(2/wheelDiameter);
                    obj.rightJointVelocity = (linear_velocity + ((obj.interWheelDistance*ang_velocity)/2))*(2/wheelDiameter);
                    
                    if(strcmp(obj.controlType,'dynamic'))
                        [res(14)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(2), obj.leftJointVelocity,obj.vrep.simx_opmode_oneshot);
                        [res(15)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(3), obj.rightJointVelocity,obj.vrep.simx_opmode_oneshot);
                        
                        
                    elseif(strcmp(obj.controlType,'kinematic'))
                        
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
                        
                        [res(16)] = obj.vrep.simxPauseCommunication(obj.clientID,1);
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(2),(p+((obj.leftJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(3),(q+((obj.rightJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[obj.robot_position(1),obj.robot_position(2), 0.0787],obj.vrep.simx_opmode_oneshot);
                        [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,-(pi/2),obj.robot_orientation(3)],obj.vrep.simx_opmode_oneshot);
                        [res(16)] = obj.vrep.simxPauseCommunication(obj.clientID,0);
                    end
                    
                elseif(strcmp(obj.robotModel,'dr20'))
                    wheelDiameter = 0.085;
                    obj.leftJointVelocity = (linear_velocity - ((obj.interWheelDistance*ang_velocity)/2))*(2/wheelDiameter);
                    obj.rightJointVelocity = (linear_velocity + ((obj.interWheelDistance*ang_velocity)/2))*(2/wheelDiameter);
                    
                    if(strcmp(obj.controlType,'dynamic'))
                        [res(14)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(2), obj.leftJointVelocity,obj.vrep.simx_opmode_oneshot);
                        [res(15)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(3), obj.rightJointVelocity,obj.vrep.simx_opmode_oneshot);
                        
                    elseif(strcmp(obj.controlType,'kinematic'))
                        
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
                        
                        [res(16)] = obj.vrep.simxPauseCommunication(obj.clientID,1);
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(2),(p+((obj.leftJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res(15)] = obj.vrep.simxSetJointPosition(obj.clientID,obj.robot_joints(3),(q+((obj.rightJointVelocity*obj.dt*2)/wheelDiameter)),obj.vrep.simx_opmode_oneshot);
                        [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[obj.robot_position(1),obj.robot_position(2), 0.1517],obj.vrep.simx_opmode_oneshot);
                        [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,0,obj.robot_orientation(3)],obj.vrep.simx_opmode_oneshot);
                        [res(16)] = obj.vrep.simxPauseCommunication(obj.clientID,0);
                    end
                end
            end
        end
        
        %% Sensor Data
        function obj = getSensorData(obj)
            switch obj.sensorID
                
                case 1
                    obj.sensor = obj.sensor.Scan(obj.vrep,obj.clientID,obj.robot,obj.controlType);
                    
                    
            end
            
        end
        
        function obj = pauseSimulation(obj)
            [res(16)] = obj.vrep.simxPauseSimulation(obj.clientID,obj.vrep.simx_opmode_oneshot);
            fprintf('Simulation Paused\n');
        end
        
        function obj = resumeSimulation(obj)
            [res(13)] = obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            frpintf('Simulation resumed');
        end
        
    end
end