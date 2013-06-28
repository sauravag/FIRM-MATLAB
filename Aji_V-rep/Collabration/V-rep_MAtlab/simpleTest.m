% Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
% marc@coppeliarobotics.com
% www.coppeliarobotics.com
% 
% -------------------------------------------------------------------
% This file is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
% 
% You are free to use/modify/distribute this file for whatever purpose!
% -------------------------------------------------------------------
%
% This file was automatically created for V-REP release V3.0.3 on April 29th 2013

% Make sure to have the server side running in V-REP!
% Start the server from a child script with following command:
% simExtRemoteApiStart(19999) -- starts a remote API server service on port 19999

function simpleTest()
	disp('Program started');
	vrep=remApi('remoteApi','extApi.h');
	clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
	if (clientID>-1)
		disp('Connected to remote API server');
		[res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait);
		if (res==vrep.simx_error_noerror)
			fprintf('Number of objects in the scene: %d\n',length(objs));
		else
			fprintf('Remote API function call returned with error code: %d\n',res);
		end
		vrep.simxFinish(clientID);
	else
		disp('Failed connecting to remote API server');
	end
	vrep.delete(); % explicitely call the destructor!
	disp('Program ended');
end
