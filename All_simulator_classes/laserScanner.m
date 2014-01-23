%%%%%%%%%%%%%%%%%%%%%%%%%% Laser Scanner Class %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author  :   Ajinkya Jain
%   email   :   jainajinkya92@gmail.com
%   Date    :   July 2013
%   Place   :   Dept. of Aerospace Engg., Texas A&M University, College
%               Station, TX, US
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Developed as a part of FIRM Toolbox for Matlab
% Useful Links:
% Further details of the functions used in this class can be seen at:
% http://www.v-rep.eu/helpFiles/en/remoteApiFunctionsMatlab.htm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


classdef laserScanner
    
    properties
        
        % response of the laser (for debugging only; if response in '-1'
        % then there is some error in execution of the command, else the
        % command was executed correctly)
        laserResponse;
        
        % Checking the working condition of laser.
        % '1' = working; '0'= not working
        status = 0;
        
        % Raw Data from single scan
        oneScan;
        rob_pos;
        rob_ori;
        
        % Refined Data for single scan
        laserData;
        robot_position;
        robot_orientation;
        
        % Time settings for synchronization
        setTime;
        getTime;
        laserTimeStamp;
        
    end
    
    
    methods
        %% Constructor
        function obj = laserScanner(vrep,clientID,robot,mode)
            %% Intializing Communication
            [obj.laserResponse(1)] = vrep.simxSetStringSignal(clientID,'request',mode,vrep.simx_opmode_oneshot);
            [obj.laserResponse(2),obj.oneScan] = vrep.simxGetStringSignal(clientID,'reply',vrep.simx_opmode_streaming);
            [obj.laserResponse(3),obj.rob_pos] = vrep.simxGetObjectPosition(clientID, robot,-1, vrep.simx_opmode_streaming);
            [obj.laserResponse(4),obj.rob_ori] = vrep.simxGetObjectOrientation(clientID,robot,-1,vrep.simx_opmode_streaming);
            
            %% Checking for initial data reception
            while(obj.status ==0)
                [obj.laserResponse(1)] = vrep.simxSetStringSignal(clientID,'request',mode,vrep.simx_opmode_oneshot);
                if (obj.laserResponse(1)==vrep.simx_error_noerror)
                    [obj.laserResponse(2),obj.oneScan] = vrep.simxGetStringSignal(clientID,'reply',vrep.simx_opmode_buffer);
                    if (obj.laserResponse(2)==vrep.simx_error_noerror)
                        obj.oneScan = [];
                        obj.status = 1;
                    end
                end
            end
        end
        
        function obj = Scan(obj,vrep,clientID,robot,mode)
            %% Setting Laser Signal
            [obj.laserResponse(1)] = vrep.simxSetStringSignal(clientID,'request',mode,vrep.simx_opmode_oneshot);
            
            if (obj.laserResponse(1)==vrep.simx_error_noerror)
                set_time = vrep.simxGetLastCmdTime(clientID);
                %                 fprintf('Signal is Set\n');
                settingtime = (set_time - obj.setTime)/1000;
                pause(settingtime);
                obj.setTime = set_time;
                
                %% Receiving response from Laser
                [obj.laserResponse(2),obj.oneScan] = vrep.simxGetStringSignal(clientID,'reply',vrep.simx_opmode_buffer);
                
                if ((obj.laserResponse(2)==vrep.simx_error_noerror))
                    obj.laserTimeStamp = vrep.simxGetLastCmdTime(clientID); %% Getting time Stamp for Communication Signal
                    %                     fprintf('Reply received\n');
                    
                    
                    
                    %% Acquiring Position
                    [obj.laserResponse(3),obj.rob_pos] = vrep.simxGetObjectPosition(clientID, robot,-1, vrep.simx_opmode_buffer);
                    [obj.laserResponse(4),obj.rob_ori] = vrep.simxGetObjectOrientation(clientID,robot,-1,vrep.simx_opmode_buffer);
                    
                    %% Setting Positions in Correct format
                    obj.robot_position(1) = obj.rob_pos(1); % Taking x out of {x,y,z}
                    obj.robot_position(2) = obj.rob_pos(2); % Taking y out of {x,y,z}
                    obj.robot_orientation = obj.rob_ori(3); % Taking gamma out of {alpha,beta,gamma}
                    
                else fprintf('Error in receiving Data\n');
                end
                
            else fprintf('Error in set signal\n');
            end
            
            %% Data Conversion from string to Numbers
            if(numel(obj.oneScan~=0))
                count =1;
                temp_count =1;
                laser_Data = zeros(1);
                
                for j= 2:(length(obj.oneScan))
                    if((obj.oneScan(j)== ',')||(obj.oneScan(j)=='}'))
                        laser_Data(count) = str2double(temp_Data);
                        count = count+1;
                        temp_count = 1;
                        continue;
                    end
                    
                    temp_Data(temp_count) = obj.oneScan(j);
                    temp_count = temp_count+1;
                end
                
                if(numel(laser_Data)>1)
                    reshapedData = reshape(laser_Data,3,(length(laser_Data)/3));
                    
                    for k = 1:length(reshapedData)
                        obj.laserData(:,k) = reshapedData(:,k);
                    end
                    
                end
            end
            %%
            pause(0.3); %% Necessary for communication. You may decrease the pause based on your requirements and System Capabilities
            
        end
    end
end