function obj  = laser_scan(obj,i)
[obj.laser_res(31)] = obj.vrep.simxSetStringSignal(obj.obj.clientID,'request','laser',obj.vrep.simx_opmode_oneshot);
if (obj.laser_res(31)==obj.vrep.simx_error_noerror)
    %             pause(0.5);
    fprintf('Signal Set\n');
    
    [obj.laser_res(32),data] = obj.vrep.simxGetStringSignal(obj.obj.clientID,'reply',obj.vrep.simx_opmode_buffer);
    if (obj.laser_res(32)==obj.vrep.simx_error_noerror)
        obj.timeStamp(i) = obj.vrep.simxGetLastCmdTime(obj.clientID);
        fprintf('No error in getsignal\n')
        [obj.laser_res(33),rob_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.bot,-1, obj.vrep.simx_opmode_buffer);
        [obj.laser_res(34),rob_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID,obj.bot,-1,obj.vrep.simx_opmode_buffer);
    end
    
    obj.robot_position(i,1) = rob_pos(1);
    obj.robot_position(i,2) = rob_pos(2);
    obj.robot_position(i,3) = rob_ori(3);
    
    
else fprintf('Error in set signal');
end
%disp(data);

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
pause(0.5);

end