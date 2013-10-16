function Test_vrep_script_delete_later()
% [res(9)] = vrep.simxGetObjectPosition(clientID,robot,-1,[position(1),position(2), 0.0957],vrep.simx_opmode_oneshot);
% [res(10)] = vrep.simxGetObjectOrientation(clientID,robot,-1,[0,0,position(3)],vrep.simx_opmode_oneshot);
%
%
% [res(11),robot_position] = vrep.simxGetObjectPosition(clientID, robot,-1, vrep.simx_opmode_oneshot_wait);
% [res(12),robot_orientation] = vrep.simxGetObjectOrientation(clientID, robot, -1,vrep.simx_opmode_oneshot_wait);
% for i=1 :17
%                     [res(14)] = vrep.simxSetJointTargetVelocity(clientID, robot_joints(i), 0,vrep.simx_opmode_streaming);
% %                     [res(15)] = vrep.simxSetJointTargetVelocity(clientID, robot_joints(3), 0,vrep.simx_opmode_streaming);
% end


%% Establish connection

% checking for correct dll file for connection
is32=exist([matlabroot,'/bin/win32'],'dir')~=0;
is64=exist([matlabroot,'/bin/win64'],'dir')~=0;
if is32
    vrep = remApi('remoteApi_32', 'extApi.h');
    disp('test')
elseif is64
    vrep = remApi('remoteApi_64', 'extApi.h');
    disp('test')
    
end
connectionStatus=0;

% Setting up the connection
while(connectionStatus==0) %Checking for the connection establishment
    clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5); % port should be taken as input. options number < 20000
    if(clientID>-1)
        fprintf('Connecton Established\n');
        connectionStatus = 1;
    else fprintf('Connection Failed...\nRetrying\n');
    end
end
scene = fullfile(pwd,'youbot_test_w_laser.ttt');%'C:\Users\Ajinkya\Documents\GitHub\FIRM-MATLAB\Aji_V-rep\laser_test_20.ttt';
% scene = fullfile(pwd,'test1.ttt');%'C:\Users\Ajinkya\Documents\GitHub\FIRM-MATLAB\Aji_V-rep\laser_test_20.ttt';




[res(3)] = vrep.simxLoadScene(clientID,scene,0,vrep.simx_opmode_oneshot_wait); % Loading the scene

%% Initialzie the simulator

numberOfObjects = 1;
% environment
% for i=1:numberOfObjects
%     [res(4), obstacles(i)] = vrep.simxGetObjectHandle(clientID,num2str(i),vrep.simx_opmode_oneshot_wait);
%
%     % changing the parameteres of the scene objects
%     [res(5)] = vrep.simxSetObjectIntParameter(clientID, obstacles(i), 3003, 0,vrep.simx_opmode_oneshot_wait);
%     [res(6)] = vrep.simxSetObjectIntParameter(clientID, obstacles(i), 3004, ~0,vrep.simx_opmode_oneshot_wait);
% end

% Initializing the robot

% Loading the Robot
%[res(7),robot] = vrep.simxLoadModel(clientID,'/home/ajinkya/Dropbox/summer13/V-REP_PRO_EDU_V3_0_3_Linux/models/robots/mobile/dr20.ttm',0,vrep.simx_opmode_oneshot_wait);
interWheelDistance = 0.254;
%% getting the handle of joints and robot
%                 [res, robot] = vrep.simxGetObjectHandle(clientID,'youBot',vrep.simx_opmode_oneshot_wait);
[res(20)] = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);

% % [res, robot] = vrep.simxGetObjectHandle(clientID,'youBot',vrep.simx_opmode_oneshot_wait);
% % [res, rollingJoint_fl] = vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_oneshot_wait);
% % [res, rollingJoint_rl] = vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_oneshot_wait);
% % [res, rollingJoint_rr] = vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_oneshot_wait);
% % [res, rollingJoint_fr] = vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_oneshot_wait);


[res, robot] = vrep.simxGetObjectHandle(clientID,'youBot',vrep.simx_opmode_oneshot_wait);
[res, rollingJoint_fl] = vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_oneshot_wait);
[res, rollingJoint_rl] = vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_oneshot_wait);
[res, rollingJoint_rr] = vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_oneshot_wait);
[res, rollingJoint_fr] = vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_oneshot_wait);




%% Set the joint velocities
forwBackVel = -0;
leftRightVel = 0;
rotVel = 0;
[res_fl] = vrep.simxSetJointTargetVelocity(clientID, rollingJoint_fl, -forwBackVel-leftRightVel-rotVel,vrep.simx_opmode_oneshot_wait);
[res_rl] = vrep.simxSetJointTargetVelocity(clientID, rollingJoint_rl, -forwBackVel+leftRightVel-rotVel,vrep.simx_opmode_oneshot_wait);
[res_rr] = vrep.simxSetJointTargetVelocity(clientID, rollingJoint_rr, -forwBackVel-leftRightVel+rotVel,vrep.simx_opmode_oneshot_wait);
[res_fr] = vrep.simxSetJointTargetVelocity(clientID, rollingJoint_fr, -forwBackVel+leftRightVel+rotVel,vrep.simx_opmode_oneshot_wait);
sensorID = 1;
controlType = 'dynamic';
sensor = laserScanner(vrep,clientID,robot,controlType);

for i = 1:20
    sensor = sensor.Scan(vrep,clientID,robot,controlType);

%     sim=sim.evolve([0.2 0]);
   
    laserData = squeeze(sensor.laserData);
    if ~isempty(laserData )
%     removing outlier point
    idx = find(abs(laserData(1,:))<10 & abs(laserData(2,:))<10);
    if i>3
        scan.x = -laserData(2,idx).*100;
        scan.y = laserData(1,idx).*100;
        new_features_set=hierarchical_feature_extracting(scan,thresholds,'new');
        axis equal
    end
    end
end


simDelete(vrep,clientID)
% simSetJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel)
% simSetJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel)
% simSetJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
% simSetJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)
%




%
% vrep.simxSetJointTargetVelocity(rollingJoint_fl,2)
% vrep.simxSetJointTargetVelocity(rollingJoint_fr,2)
% simSetJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
% simSetJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)
%
%
%
% simSetJointTargetVelocity(rollingJoint_fl,-forwBackVel-leftRightVel-rotVel)
% simSetJointTargetVelocity(rollingJoint_fr,-forwBackVel+leftRightVel-rotVel)
% simSetJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
% simSetJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)
% %



% 	wheelJoints[1]=simGetObjectHandle('rollingJoint_fl')
% 	wheelJoints[2]=simGetObjectHandle('rollingJoint_rl')
% 	wheelJoints[3]=simGetObjectHandle('rollingJoint_rr')
% 	wheelJoints[4]=simGetObjectHandle('rollingJoint_fr')


%Handles for various parts of robot
% % % % % % % [res(8), robot_joints] = vrep.simxGetObjects(clientID, vrep.sim_object_joint_type,vrep.simx_opmode_oneshot_wait);
% % % % % % % [errorCode, handles, intData, floatData, stringData]=simxGetObjectGroupData( clientID, vrep.sim_appobj_object_type, 0, vrep.simx_opmode_oneshot_wait)
% % % % % % % %Intializing the Environment
% % % % % % % [res(20)] = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
% % % % % % % [res(14)] = vrep.simxSetJointTargetVelocity(clientID, robot_joints(2), 0,vrep.simx_opmode_streaming);
% % % % % % % [res(15)] = vrep.simxSetJointTargetVelocity(clientID, robot_joints(3), 0,vrep.simx_opmode_streaming);
% % % % % % %
% % % % % % % sensorID = 1;
% % % % % % % sensor = laserScanner(vrep,clientID,robot,controlType);
% % % % % % %
% % % % % % %
% % % % % % % [res(9)] = vrep.simxSetObjectPosition(clientID,robot,-1,[position(1),position(2), 0.0957],vrep.simx_opmode_oneshot);
% % % % % % % [res(10)] = vrep.simxSetObjectOrientation(clientID,robot,-1,[0,0,position(3)],vrep.simx_opmode_oneshot);
% % % % % % %
% % % % % % % [res(11),robot_position] = vrep.simxGetObjectPosition(clientID, robot,-1, vrep.simx_opmode_oneshot_wait);
% % % % % % % [res(12),robot_orientation] = vrep.simxGetObjectOrientation(clientID, robot, -1,vrep.simx_opmode_oneshot_wait);
% % % % % % %
% % % % % % % [res(14)] = vrep.simxSetJointTargetVelocity(clientID, robot_joints(2), leftJointVelocity,vrep.simx_opmode_oneshot);
% % % % % % % [res(15)] = vrep.simxSetJointTargetVelocity(clientID, robot_joints(3), rightJointVelocity,vrep.simx_opmode_oneshot);
% % % % % % % for i=1:17
% % % % % % %     [res(14)] = vrep.simxSetJointTargetVelocity(clientID, robot_joints(i), leftJointVelocity,vrep.simx_opmode_oneshot);
% % % % % % %
% % % % % % % end


end



function vrep = simPause(vrep,clientID)
[res(16)] = vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot);
fprintf('Simulation Paused\n');
end

%% Resume Simulation
function vrep = simResume(vrep,clientID)
[res(13)] = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
frpintf('Simulation resumed');
end

%% Stop Simulation
function vrep = simStop(vrep,clientID)
[res(19)] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait);
fprintf('Simulation Stopped\n');
end
function vrep = simDelete(vrep,clientID)

fprintf('Stopping Simulation and Calling Destructor\n');
[res(19)] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait); % Stopping Simulation
[res(18)] = vrep.simxCloseScene(clientID, vrep.simx_opmode_oneshot_wait); % Closing the loaded V-Rep scene
vrep.simxFinish(clientID);  % Ending the connection
vrep.delete();
fprintf('Simulation Ended\n');
end