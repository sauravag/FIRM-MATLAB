% clear classes;clear variables;
close all;clc;
clear all
cd('C:\Users\Ajinkya\Dropbox\FIRM_toolbox_ver_current (copy)\')
addpath(genpath(pwd))
tic
% pauseTime = 2;
% wheelRadi = 0.085/2 ;%% (meter)
% velRPM = 0.5*23.5294;%%

disp('Program started');
vrep = remApi('remoteApi','extApi.h');
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5)

if (clientID>-1)
    disp('Connected to remote API server');
    
    [response] = vrep.simxLoadScene(clientID,'C:\Users\Ajinkya\Desktop\Summers13\V-rep_matlab\Scenes\laser_test.ttt',0,vrep.simx_opmode_oneshot_wait);
    pause(1);
    fprintf('Load command sent...\n');
    
    [res, bot] = vrep.simxGetObjectHandle(clientID,'dr12',vrep.simx_opmode_oneshot_wait);
    [res, laser] = vrep.simxGetObjectHandle(clientID,'laser',vrep.simx_opmode_oneshot_wait);
    

    [res] = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
%     pause(0.5);
    disp('Simulation Started');
    state = zeros(100,1);
%     Data = cell(100,1);
    detectedObject = zeros(100,1);
    %     normalVector = zeros(100,1);
    
    for i=1:300
        %         [res,bot_collision]= vrep.simxGetCollisionHandle(clientID,'dr20',vrep.simx_opmode_oneshot_wait);
        [res,state,sensorData, detectedObject,normalVector]=vrep.simxReadProximitySensor(clientID,laser,vrep.simx_opmode_streaming);
        Data(i,:) = sensorData(:);
        pause(0.05);
    end
    
    [res1] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait);
    
    FO_DELETE=1; vrep.simxFinish(clientID);vrep.delete();
    disp('Program Ended');
end
toc