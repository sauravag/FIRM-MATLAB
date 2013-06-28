
function obstacles()
% 	disp('Program started');
	vrep = remApi('remoteApi','extApi.h');
	clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5)
    
	if (clientID>-1)
		disp('Connected to remote API server');
% 		[res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait);
% 		if (res==vrep.simx_error_noerror)
% 			fprintf('Number of objects in the scene: %d\n',length(objs));
% 		else
% 			fprintf('Remote API function call returned with error code: %d\n',res);
% 		end

%         [res] = vrep.simxTransferFile(clientID,'/home/ajinkya/summer13/V-rep_Matlab/environment.obj','obj1.obj',20,vrep.simx_opmode_oneshot_wait);
%         pause(1);

        % CREATING A DUMMY OBJECT
        %[res, dummy]= simxCreateDummy(clientID,2,[],simx_opmode_oneshot_wait);
        
        pause(1)
        [res, cube] = vrep.simxGetObjectHandle(clientID,'cube',vrep.simx_opmode_oneshot_wait);
        [res, bot] = vrep.simxGetObjectHandle(clientID,'dr12',vrep.simx_opmode_oneshot_wait);
        
        %%Uncomment this for defining obstacles in V-rep
%         [res1] = vrep.simxSetObjectIntParameter(clientID, cube, 3003, ~0,vrep.simx_opmode_oneshot_wait);
%         [res2] = vrep.simxSetObjectIntParameter(clientID, cube, 3004, ~0,vrep.simx_opmode_oneshot_wait);


        %% Defining position of the Bot in V-rep
        for i= 1:len(action.x)
            [res] = vrep.simxSetObjectPosition(clientID,bot,-1,{action.x(1,i),action.x(2,i),0},vrep.simx_opmode_oneshot);
            [res1] = vrep.simxSetObjectOrientation(clientID,bot,-1,{0,0,action.x(3,i)},vrep.simx_opmode_oneshot);
        end
      
		vrep.simxFinish(clientID);
	else
		disp('Failed connecting to remote API server');
	end
	vrep.delete(); % explicitely call the destructor!
	disp('Program ended');
end
    