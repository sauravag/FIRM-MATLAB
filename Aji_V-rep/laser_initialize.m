function obj = laser_initialize(obj)

[obj.laser_res(31)] = vrep.simxSetStringSignal(obj.clientID,'request','laser',obj.vrep.simx_opmode_oneshot);
[obj.laser_res(32),data] = vrep.simxGetStringSignal(obj.clientID,'reply',obj.vrep.simx_opmode_streaming);
[obj.laser_res(33),rob_pos] = vrep.simxGetObjectPosition(obj.clientID, bot,-1, obj.vrep.simx_opmode_streaming);
[obj.laser_res(34),rob_ori] = vrep.simxGetObjectOrientation(obj.clientID,bot,-1,obj.vrep.simx_opmode_streaming);

end