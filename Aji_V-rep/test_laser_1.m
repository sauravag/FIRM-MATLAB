% clear classes;clear variables;
close all;clc;
clear all
%  cd('C:\Users\Ajinkya\Dropbox\FIRM_toolbox_ver_current (copy)\')
addpath(genpath(pwd))
tic

%% Connection Making
disp('Program started');
vrep = remApi('remoteApi','extApi.h');
clientID = vrep.simxStart('127.0.0.1',19999,true,false,5000,5);

%% Operations
if (clientID>-1)
    disp('Connected to remote API server');
    
    [response] = vrep.simxLoadScene(clientID,'C:\Users\Amirhossein\Dropbox\FIRM_toolbox_ver_current\Aji_V-rep\laser_test1.ttt',0,vrep.simx_opmode_oneshot_wait);
    pause(1);
    
    [res, bot] = vrep.simxGetObjectHandle(clientID,'dr12',vrep.simx_opmode_oneshot_wait);
    
    [res] = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    
    disp('Simulation Started');
    pause(2);
    
    %% Streaming initialization:
    %      [err,laserScannerData,dataSize] = vrep.simxGetAndClearStringSignal(clientID,'ScannerData',vrep.simx_opmode_streaming);
    %     vrep.simxGetAndClearStringSignal(clientID,'ScannerData',vrep.simx_opmode_streaming);
    
    %% Data Acquisition
    
    %     for i = 1:10
    %         [res,replyData] = vrep.simxQuery(clientID,'request','laser','reply',5000);
    %         if (res==vrep.simx_error_noerror)
    %             fprintf('The reply is: %s\n------------------\n',replyData);
    %             [response,data] = vrep.simxGetFloatSignal(clientID,'ScannerData',vrep.simx_opmode_oneshot_wait);
    %             if(response == vrep.simx_error_initialize_error_flag)
    % %                 fprintf('initialize_error_flag');
    %             end
    %
    % %             disp(data);
    %         end
    %     end
     [err] = vrep.simxSetStringSignal(clientID,'request','laser',vrep.simx_opmode_oneshot);
     [res,data] = vrep.simxGetStringSignal(clientID,'reply',vrep.simx_opmode_streaming);
     [res,pos] = vrep.simxGetObjectPosition(clientID, bot,-1, vrep.simx_opmode_streaming);
     [res,ori] = vrep.simxGetObjectOrientation(clientID,bot,-1,vrep.simx_opmode_streaming);

       
     
     iterations = 100;
     FinalData = zeros(iterations,3,100);
     
     for i =1:iterations
%         [res] = vrep.simxClearStringSignal(clientID,'request',vrep.simx_opmode_oneshot);
        [error] = vrep.simxSetStringSignal(clientID,'request','laser',vrep.simx_opmode_oneshot);
        if (error==vrep.simx_error_noerror)
%             pause(0.5);
            fprintf('Signal Set\n');
            
            [response,data] = vrep.simxGetStringSignal(clientID,'reply',vrep.simx_opmode_buffer);
             if (response==vrep.simx_error_noerror)
                 timeStamp = vrep.simxGetLastCmdTime(clientID);
                 fprintf('No error in getsignal\n')
                [res,pos] = vrep.simxGetObjectPosition(clientID, bot,-1, vrep.simx_opmode_buffer);
                [res,ori] = vrep.simxGetObjectOrientation(clientID,bot,-1,vrep.simx_opmode_buffer);
             end
             robot_position(i,1) = pos(1);
             robot_position(i,2) = pos(2);
             robot_position(i,3) = ori(3);
             
             
        else fprintf('Error in set signal');
        end
        %disp(data);
            
        %% Data Conversion from string to Numbers
        if(numel(data~=0))
            count =1;
            temp_count =1;
            
            for j= 2:(length(data))
                if((data(j)== ',')||(data(j)=='}'))
                    laserData(count) = str2double(temp_data);
                    count = count+1;
                    temp_count = 1;
                    continue;
                end
%                 fprintf('IN for loop');
                temp_data(temp_count) = data(j);
                temp_count = temp_count+1;
            end
            
%             if(laserData)
                reshapedData = reshape(laserData,3,(length(laserData)/3));
                
               for k = 1:length(reshapedData) 
                FinalData(i,:,k) = reshapedData(:,k);
               end
                time_stamp(i) = timeStamp;
%             end
        end
        %% 
         pause(0.5);
            
        fprintf('\n-----------------\n'); 
    end
    
    
    
    
    %      [res] = vrep.simxSetStringSignal(clientID,'ScannerData','laser1',vrep.simx_opmode_streaming);
    %      pause(1);
    %      [response,data] = vrep.simxGetStringSignal(clientID,'ScannerData',vrep.simx_opmode_oneshot_wait);
    
    %     for i=1:100
    %         [err,laserScannerData]=vrep.simxGetAndClearStringSignal(clientID,'ScannerData',vrep.simx_opmode_oneshot_wait);
    %         %         vrep.simxGetAndClearStringSignal(clientID,'ScannerData',vrep.simx_opmode_buffer);
    %         %         [errorCode,signalValue,signalLength] = vrep.simxGetStringSignal(clientID,'ScannerData',vrep.simx_opmode_streaming);
    %         dataSize = length(laserScannerData);
    %         if (err==vrep.simx_error_noerror)
    %             dat = setDataType(laserScannerData,'char','int8Ptr');
    %             scannerData(i,:) = laserScannerData.value(:);
    % %             [res] = vrep.simxSetStringSignal(clientID,'ScannerData',laserScannerData,vrep.simx_opmode_streaming);
    %         end
    %         pause(0.05);
    %     end
    %
    
    %% Finishing Communication
    [res1] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait);
    
    FO_DELETE=1; vrep.simxFinish(clientID);vrep.delete();
    disp('Program Ended');
    
else
    vrep.simxFinish(clientID);vrep.delete();
end
toc