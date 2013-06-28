classdef laserScanner < vrep_interface
    
    properties
        laserResponse;
        
    end
    
    methods
        function obj = laserScanner(obj) %% Constructor
            %% Intializing Communication
            [obj.laserResponse(1)] = vrep.simxSetStringSignal(obj.clientID,'request','laser',obj.vrep.simx_opmode_oneshot);
            [obj.laserResponse(2),data] = vrep.simxGetStringSignal(obj.clientID,'reply',obj.vrep.simx_opmode_streaming);
            [obj.laserResponse(3),rob_pos] = vrep.simxGetObjectPosition(obj.clientID, bot,-1, obj.vrep.simx_opmode_streaming);
            [obj.laserResponse(4),rob_ori] = vrep.simxGetObjectOrientation(obj.clientID,bot,-1,obj.vrep.simx_opmode_streaming);
        end
        
        function obj = laserScan(obj,i)
            %% Setting Laser Signal
            [obj.laserResponse(1)] = obj.vrep.simxSetStringSignal(obj.obj.clientID,'request','laser',obj.vrep.simx_opmode_oneshot);
            
            if (obj.laserResponse(1)==obj.vrep.simx_error_noerror)
                fprintf('Signal is Set\n');
                
                %% Receiving response from Laser
                [obj.laserResponse(2),data] = obj.vrep.simxGetStringSignal(obj.obj.clientID,'reply',obj.vrep.simx_opmode_buffer);
                
                if (obj.laserResponse(2)==obj.vrep.simx_error_noerror)
                    obj.timeStamp(i) = obj.vrep.simxGetLastCmdTime(obj.clientID); %% Getting time Stamp for Communication Signal
                    fprintf('Reply received\n')
                    
                    %% Acquiring Position
                    [obj.laserResponse(3),rob_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.bot,-1, obj.vrep.simx_opmode_buffer);
                    [obj.laserResponse(4),rob_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID,obj.bot,-1,obj.vrep.simx_opmode_buffer);
                end
                
                %% Setting Positions in Correct format
                obj.robot_position(i,1) = rob_pos(1);
                obj.robot_position(i,2) = rob_pos(2);
                obj.robot_position(i,3) = rob_ori(3);
                
                
            else fprintf('Error in set signal');
            end
            
            %% Data Conversion from string to Numbers
            if(numel(data~=0))
                count =1;
                temp_count =1;
                laser_data = zeros(1);
                
                for j= 2:(length(data))
                    if((data(j)== ',')||(data(j)=='}'))
                        laser_data(count) = str2double(temp_data);
                        count = count+1;
                        temp_count = 1;
                        continue;
                    end
                    
                    temp_data(temp_count) = data(j);
                    temp_count = temp_count+1;
                end
                
                if(numel(laser_data)>1)
                    reshapedData = reshape(laser_data,3,(length(laser_data)/3));
                    
                    for k = 1:length(reshapedData)
                        obj.laserData(i,:,k) = reshapedData(:,k);
                    end
                    
                end
            end
            %%
            pause(0.5); %% Necessary for communication. You may decrease the pause based on your requirements and System Capabilities
            
        end
    end
end