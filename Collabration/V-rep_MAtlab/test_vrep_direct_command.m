% clear classes;clear variables;
% close all;clc;
cd('C:\Users\Amirhossein\Dropbox\FIRM_toolbox_ver_current\')
addpath(genpath(pwd))

pauseTime =10;
wheelRadi = 0.085/2 ;%% (meter)
velRPM = 23.5294;%%

disp('Program started');
vrep = remApi('remoteApi','extApi.h');
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5)

if (clientID>-1)
    disp('Connected to remote API server');
    
    [response] = vrep.simxLoadScene(clientID,'C:\Program Files (x86)\V-REP3\V-REP_PRO_EDU\scenes\test_vrep.ttt',0,vrep.simx_opmode_oneshot_wait);
    pause(1);
    fprintf('Load command sent...\n');
    
    [res, bot] = vrep.simxGetObjectHandle(clientID,'dr20',vrep.simx_opmode_oneshot_wait);
    [res, left_joint] = vrep.simxGetObjectHandle(clientID,'dr20_leftWheelJoint_',vrep.simx_opmode_oneshot_wait);
    [res, right_joint] = vrep.simxGetObjectHandle(clientID,'dr20_rightWheelJoint_',vrep.simx_opmode_oneshot_wait);
    [res, robot_joints] = vrep.simxGetObjects(clientID, vrep.sim_object_joint_type,vrep.simx_opmode_oneshot_wait);
    
    [res] = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    simulationTime = 10; % second
    vrep_simulator_period = 0.05 ; % sampling time of the simulator
    nStepNeeded = round(simulationTime/vrep_simulator_period)
    [res] = vrep.simxSetObjectPosition(clientID,bot,-1,[0,0,0.1517],vrep.simx_opmode_oneshot);
    
    [res1] = vrep.simxSetObjectOrientation(clientID,bot,-1,[0,0,0],vrep.simx_opmode_oneshot);
    [res] = vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot);

%     for i=1:nStepNeeded
        
        vrep.simxPauseCommunication(clientID,1);
        tic
        [res] = vrep.simxSetJointTargetVelocity(clientID,left_joint,velRPM, vrep.simx_opmode_oneshot_wait);
        a= toc
        tic
        [res] = vrep.simxSetJointTargetVelocity(clientID,right_joint,velRPM, vrep.simx_opmode_oneshot_wait);
        a=toc
        vrep.simxPauseCommunication(clientID,0);
            [res] = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);

        pause(10)
        %         pos1 = vrep.simxGetObjectPosition(clientID,bot,-1,vrep.simx_opmode_oneshot_wait)
        %         pos2 = vrep.simxGetObjectOrientation(clientID,bot,-1,vrep.simx_opmode_oneshot_wait)
        %     [errorCode,pingTime]=vrep.simxGetPingTime(clientID);
        
        
%     end
    %     FO_DELETE=1; vrep.simxFinish(clientID);vrep.delete();
    %     disp('I am Abbas Ghaderi')
    %     [res1] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait);
    %     fprintf('Simulation should stop now....\n');
    %     vrep.simxFinish(clientID);
    
else
    disp('Failed connecting to remote API server');
end
[res] = vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot);
pos1 = vrep.simxGetObjectPosition(clientID,bot,-1,vrep.simx_opmode_oneshot_wait)
pos2 = vrep.simxGetObjectOrientation(clientID,bot,-1,vrep.simx_opmode_oneshot_wait)
[res1] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait);


FO_DELETE=1; vrep.simxFinish(clientID);vrep.delete();
disp('I am Abbas Ghaderi')
%     [res1] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait);
fprintf('Simulation should stop now....\n');
vrep.simxFinish(clientID);



% vrep.delete(); % explicitely call the destructor!
disp('Program ended');

