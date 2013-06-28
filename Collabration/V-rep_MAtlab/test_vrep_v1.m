% clear classes;clear variables;
% close all;clc;
cd('C:\Users\Ajinkya\Dropbox\FIRM_toolbox_ver_current (copy)\')
addpath(genpath(pwd))

pauseTime =11;
wheelRadi = 0.085/2 ;%% (meter)
velRPM = 23.5294;%% 

disp('Program started');
vrep = remApi('remoteApi','extApi.h');
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5)

if (clientID>-1)
    disp('Connected to remote API server');
    
    [response] = vrep.simxLoadScene(clientID,'C:\Users\Ajinkya\Desktop\Summers13\V-rep_matlab\Scenes\test.ttt',0,vrep.simx_opmode_oneshot_wait);
    pause(1);
    fprintf('Load command sent...\n');
    
    [res, bot] = vrep.simxGetObjectHandle(clientID,'dr20',vrep.simx_opmode_oneshot_wait);
    [res, left_joint] = vrep.simxGetObjectHandle(clientID,'dr20_leftWheelJoint_',vrep.simx_opmode_oneshot_wait);
    [res, right_joint] = vrep.simxGetObjectHandle(clientID,'dr20_rightWheelJoint_',vrep.simx_opmode_oneshot_wait);
    [res, robot_joints] = vrep.simxGetObjects(clientID, vrep.sim_object_joint_type,vrep.simx_opmode_oneshot_wait);
    
    [res] = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    
    [res] = vrep.simxSetObjectPosition(clientID,bot,-1,[0,0,0.1517],vrep.simx_opmode_oneshot);
    
    [res1] = vrep.simxSetObjectOrientation(clientID,bot,-1,[0,0,0],vrep.simx_opmode_oneshot);
    
    
    
    [res] = vrep.simxSetJointTargetVelocity(clientID,left_joint,velRPM, vrep.simx_opmode_streaming);
    [res] = vrep.simxSetJointTargetVelocity(clientID,right_joint,velRPM, vrep.simx_opmode_streaming);
    pause(pauseTime);
    [res] = vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot);
    pos1 = vrep.simxGetObjectPosition(clientID,bot,-1,vrep.simx_opmode_oneshot_wait)
    pos2 = vrep.simxGetObjectOrientation(clientID,bot,-1,vrep.simx_opmode_oneshot_wait)
    
    
    
    FO_DELETE=1; vrep.simxFinish(clientID);vrep.delete();
    disp('I am Abbas Ghaderi')
    
    [res1] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait);
    fprintf('Simulation should stop now....\n');
    
    vrep.simxFinish(clientID);
    
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % explicitely call the destructor!
disp('Program ended');

