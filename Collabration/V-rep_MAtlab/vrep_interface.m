classdef vrep_interface %< SimulatorInterface
    properties (Constant)
        vrep = remApi('remoteApi', 'extApi.h');
    end
      
    properties
        clientID;
        scene;
        obstacles;
        numberOfObjects=1;
        robot;
        robot_joints;
        robot_position;
        robot_orientation;
        robot_body;
        SizeOfRobot;
        res;
               
        
    end
    
    methods
        function obj = vrep_interface()  % Constructor
            
            %obj = obj@SimulatorInterface();
            
           % Setting up the connection
            obj.clientID = obj.vrep.simxStart('127.0.0.1',19999,true,true,5000,5); % port should be taken as input. options number < 20000 
                     
            if(obj.clientID>-1)
                fprintf('Connecton Established\n');
            else fprintf('Connection Failed...\nRetry\n');% Try to make a call to destructor and retry making the connection
                obj.vrep.delete();
            end
        end
        
        function delete(obj)
            [res(19)] = obj.vrep.simxStopSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot_wait);
            [res(18)] = obj.vrep.simxCloseScene(obj.clientID, obj.vrep.simx_opmode_oneshot_wait);
            obj.vrep.simxFinish(obj.clientID);
            obj.vrep.delete();
        end
    end
       
    methods 
        function obj = simInitialize(obj)
            % Transfer the .obj file (take the path from .obj file generator)
%             [res(1)] = obj.vrep.simxTransferFile(obj.clientID,'/home/ajinkya/Dropbox/FIRM_toolbox_ver_current/cube1.obj','cube1.obj',20,obj.vrep.simx_opmode_oneshot_wait);
%                         
%             % Initializing scene saving
%            [res(2),replyData] = obj.vrep.simxQuery(obj.clientID,'request','save','reply',5000);
%            if (res==obj.vrep.simx_error_noerror)
%                fprintf('scene saved\n');
%                obj.scene = replyData;
%                fprintf('scene saved at location: %s\n', obj.scene);
%            end
                  
            %obj.scene = '/home/ajinkya/Dropbox/summer13/V-REP_PRO_EDU_V3_0_3_Linux/scenes/myscene.ttt';
            % Loading the environment and obstacles
            [res(3)] = obj.vrep.simxLoadScene(obj.clientID,obj.scene,0,obj.vrep.simx_opmode_oneshot_wait); 
          
            for i=1:obj.numberOfObjects
                [res(4), obj.obstacles(i)] = obj.vrep.simxGetObjectHandle(obj.clientID,num2str(i),obj.vrep.simx_opmode_oneshot_wait);
                
                % changing the parameteres of the objects
                [res(5)] = obj.vrep.simxSetObjectIntParameter(obj.clientID, obj.obstacles(i), 3003, 0,obj.vrep.simx_opmode_oneshot_wait);
                [res(6)] = obj.vrep.simxSetObjectIntParameter(obj.clientID, obj.obstacles(i), 3004, ~0,obj.vrep.simx_opmode_oneshot_wait);
            end
        end
            
        
        function obj = SetRobot(obj,position)
             %Loading the Robot
            [res(7),obj.robot] = obj.vrep.simxLoadModel(obj.clientID,'/home/ajinkya/Dropbox/summer13/V-REP_PRO_EDU_V3_0_3_Linux/models/robots/mobile/dr20.ttm',0,obj.vrep.simx_opmode_oneshot_wait);
            
            % Dimensions of Robot
            [res(17),obj.robot_body] = obj.vrep.simxGetObjectHandle(obj.clientID, 'dr20_body_',obj.vrep.simx_opmode_oneshot_wait);
            [res(18),y_min] = obj.vrep.simxGetObjectFloatParameter(obj.clientID, obj.robot_body, 16, obj.vrep.simx_opmode_oneshot_wait);
            [res(19),y_max] = obj.vrep.simxGetObjectFloatParameter(obj.clientID, obj.robot_body, 19, obj.vrep.simx_opmode_oneshot_wait);
             
            obj.SizeOfRobot = y_max - y_min;
            
            
            %Handles for various parts of robot
            [res(8), obj.robot_joints] = obj.vrep.simxGetObjects(obj.clientID, obj.vrep.sim_object_joint_type,obj.vrep.simx_opmode_oneshot_wait);
            
            %Setting initial position
            [res(9)] = obj.vrep.simxSetObjectPosition(obj.clientID,obj.robot,-1,[position.val(1),position.val(2), 0.1517],obj.vrep.simx_opmode_oneshot_wait);
            [res(10)] = obj.vrep.simxSetObjectOrientation(obj.clientID,obj.robot,-1,[0,0,position.val(3)],obj.vrep.simx_opmode_oneshot_wait);
                
        end
        
        
        function obj = getRobot(obj)
           [res(11),obj.robot_position] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.robot,-1, obj.vrep.simx_opmode_streaming);
           [res(12),obj.robot_orientation] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.robot, -1,obj.vrep.simx_opmode_streaming);
        end
        
        
        function refresh(obj)
        end
                
        function evolve(obj,control)
            % Starting simulation
            [res(13)] = obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);            
            
            linear_velocity = control(1);
            ang_velocity = control(2);
            
            leftJointVelocity = linear_velocity + ((obj.SizeOfRobot*ang_velocity)/2);
            rightJointVelocity = linear_velocity - ((obj.SizeOfRobot*ang_velocity)/2);
            
            [res(14)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(2), leftJointVelocity,obj.vrep.simx_opmode_oneshot);
            [res(15)] = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(3), rightJointVelocity,obj.vrep.simx_opmode_oneshot);
       
%             pause(user_data_class.par.motion_model_parameters.dt);
             % pause(2);
            [res(16)] = obj.vrep.simxPauseSimulation(obj.clientID,obj.vrep.simx_opmode_oneshot_wait);
                                   
        end
    end
end