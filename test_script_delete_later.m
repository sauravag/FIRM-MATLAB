clear classes;clear variables;close all;clc;
%load myfilebrkpnts;dbstop(s)
addpath(genpath('c:\users\amirhossein\desktop\mywork_tamu\firm_with_simulated_laser\ecmr paper all things\ecmr_paper_codes\completed_after_paper_trends\for ecmr\'))
addpath('./external/rvctools/'); % we need rvctoolbox to run rrt
startup_rvc;
% dbstop if error
thresholds=default_thresholds_func()
% parameters
user_data = user_data_class; % the object user_data will never be used. this line only cause the "constant" properties of the "user_data_class" class to be initialized.

robot_init = [0 0 pi/12]';
sim = simulator();
sim = sim.initialize();
% [ errorcode]=sim.vrep.simxsetmodelproperty( sim.clientid, sim.robot, sim.vrep.sim_modelproperty_not_dynamic,sim.vrep.simx_opmode_oneshot_wait)

% [ errorcode]=sim.vrep.simxsetmodelproperty( sim.clientid, sim.robot, sim.vrep.sim_modelproperty_not_dynamic,sim.vrep.simx_opmode_oneshot_wait)

sim = sim.setRobot(robot_init);
mm = motionmodel_class;
robot_final = [10 20 pi/4]';


traj = mm.generate_open_loop_point2point_traj(robot_init,robot_final);
% for i =1:size(traj.u,2)
%
%     %     [res(10)] = sim.vrep.simxsetobjectorientation(sim.clientid,sim.robot,-1,[-pi/2,0.5,-pi/2],sim.vrep.simx_opmode_oneshot);
%     %                 [res(9)] = sim.vrep.simxsetobjectposition(sim.clientid,sim.robot,-1,[traj.x(1,i),traj.x(2,i), 0.0957],sim.vrep.simx_opmode_oneshot);
%
%     sim = sim.evolve(traj.u(:,i))
%     %     sim = sim.setrobot(traj.x(:,i))
%     pause(0.3)
% end


for i =1:size(traj.u,2)
    
    %         [res(10)] = sim.vrep.simxsetobjectorientation(sim.clientid,sim.robot,-1,[-pi/2,traj.x(3,i),-pi/2],sim.vrep.simx_opmode_oneshot);
    %                     [res(9)] = sim.vrep.simxsetobjectposition(sim.clientid,sim.robot,-1,[traj.x(1,i),traj.x(2,i), 0.0957],sim.vrep.simx_opmode_oneshot);
    
%     sim = sim.evolve(traj.u(:,i));
         sim = sim.setRobot(traj.x(:,i+1));
%     robot_ = sim.getRobot();
%     disp([robot_.robot_position(1),robot_.robot_position(2),robot_.robot_orientation(2)])
%     disp([traj.x(:,i+1)'])
%     disp('---------------------')
%     %     sim = sim.setrobot(traj.x(:,i))
%     pause(0.7)
    
    sim = sim.getSensorData();
    
    %     sim=sim.evolve([0.2 0]);
    
    laserdata = squeeze(sim.sensor.laserData);
    if ~isempty(laserdata )
        %     removing outlier point
        idx = find(abs(laserdata(1,:))<10 & abs(laserdata(2,:))<10);
        if rem(i,5)==0
            scan.x = -laserdata(2,idx).*100;
            scan.y = laserdata(1,idx).*100;
            new_features_set=hierarchical_feature_extracting(scan,thresholds,'new');
            axis equal
        end
        
    end
end    
    
    
    
    
    
    
    %         0    0.0500    0.1000    0.1500    0.2000    0.2500    0.3000    0.3500    0.4000    0.4500    0.5000
    %          0    0.1000    0.2000    0.3000    0.4000    0.5000    0.6000    0.7000    0.8000    0.9000    1.0000
    %     0.2618    0.2880    0.3142    0.3403    0.3665    0.3927    0.4189    0.4451    0.4712    0.4974    0.5236
    
    
    
    [a,b] = sim.vrep.simxgetmodelproperty( sim.clientid, sim.robot,sim.vrep.simx_opmode_oneshot_wait)
    sim = sim.simDelete();
    
    sim = sim.simdelete();
    [a,b] = sim.vrep.simxgetmodelproperty( sim.clientid, sim.robot,sim.vrep.simx_opmode_oneshot_wait)
    [res(10)] = sim.vrep.simxsetobjectorientation(sim.clientid,-1,sim.robot,[sim.robot_orientation(3),0,0],sim.vrep.simx_opmode_oneshot);
    
    [ errorcode]=sim.vrep.simxsetmodelproperty( sim.clientid, sim.robot, sim.vrep.sim_modelproperty_not_dynamic,sim.vrep.simx_opmode_oneshot_wait)
    
    [res(10)] = sim.vrep.simxsetobjectorientation(sim.clientid,sim.robot,-1,[-pi/2,0.5,-pi/2],sim.vrep.simx_opmode_oneshot);
    
    [a,b] = obj.vrep.simxgetmodelproperty( obj.clientid, obj.robot,obj.vrep.simx_opmode_oneshot_wait)
    a=obj.vrep.simxgetobjectvelocity(obj.clientid,  obj.robot_joints.rollingjoint_fl,obj.vrep.simx_opmode_oneshot_wait)
    % % % % % % % % % % % % % clear time1
    % % % % % % % % % % % % % [errorcode]=sim.vrep.simxpausesimulation(sim.clientid,sim.vrep.simx_opmode_oneshot_wait)
    % % % % % % % % % % % % % [ errorcode]=sim.vrep.simxstartsimulation( sim.clientid, sim.vrep.simx_opmode_oneshot_wait)
    % % % % % % % % % % % % % robot_init = state([0 0 0]);
    % % % % % % % % % % % % % sim = sim.setrobot(robot_init);
    % % % % % % % % % % % % %
    % % % % % % % % % % % % % time1=sim.vrep.simxgetlastcmdtime(sim.clientid)
    % % % % % % % % % % % % % [res, sim.robot] = sim.vrep.simxgetobjecthandle(sim.clientid,'youbot',sim.vrep.simx_opmode_oneshot_wait);
    % % % % % % % % % % % % %
    % % % % % % % % % % % % % % [ errorcode]=sim.vrep.simxsetmodelproperty( sim.clientid, objecthandle,number prop,number operationmode)
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % % [res, rollingjoint_fl] = sim.vrep.simxgetobjecthandle(sim.clientid,'rollingjoint_fl',sim.vrep.simx_opmode_oneshot_wait); %1-->1
    % % % % % % % % % % % % % [res, rollingjoint_fr] = sim.vrep.simxgetobjecthandle(sim.clientid,'rollingjoint_fr',sim.vrep.simx_opmode_oneshot_wait); %4-->2
    % % % % % % % % % % % % % [res, rollingjoint_rl] = sim.vrep.simxgetobjecthandle(sim.clientid,'rollingjoint_rl',sim.vrep.simx_opmode_oneshot_wait); %2-->3
    % % % % % % % % % % % % % [res, rollingjoint_rr] = sim.vrep.simxgetobjecthandle(sim.clientid,'rollingjoint_rr',sim.vrep.simx_opmode_oneshot_wait); %3-->4
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % % %% set the joint velocities
    % % % % % % % % % % % % % control = [0 0 0 0 ]
    % % % % % % % % % % % % % vel_w_fl = control(1);
    % % % % % % % % % % % % % vel_w_fr = control(2);
    % % % % % % % % % % % % % vel_w_rl = control(3);
    % % % % % % % % % % % % % vel_w_rr = control(4);
    % % % % % % % % % % % % %
    % % % % % % % % % % % % % [res_fl] = sim.vrep.simxsetjointtargetvelocity(sim.clientid, rollingjoint_fl,vel_w_fl,sim.vrep.simx_opmode_oneshot_wait);%1-->1
    % % % % % % % % % % % % % [res_fr] = sim.vrep.simxsetjointtargetvelocity(sim.clientid, rollingjoint_fr, vel_w_fr,sim.vrep.simx_opmode_oneshot_wait);%4-->2
    % % % % % % % % % % % % % [res_rl] = sim.vrep.simxsetjointtargetvelocity(sim.clientid, rollingjoint_rl,vel_w_rl,sim.vrep.simx_opmode_oneshot_wait);%2-->3
    % % % % % % % % % % % % % [res_rr] = sim.vrep.simxsetjointtargetvelocity(sim.clientid, rollingjoint_rr, vel_w_rr,sim.vrep.simx_opmode_oneshot_wait);%3-->4
    % % % % % % % % % % % % % tic
    % % % % % % % % % % % % % sim.vrep.simxgetlastcmdtime(sim.clientid)
    % % % % % % % % % % % % % time1
    % % % % % % % % % % % % % while sim.vrep.simxgetlastcmdtime(sim.clientid) -time1<6000
    % % % % % % % % % % % % %     [ errorcode, pingtime]=sim.vrep.simxgetpingtime(sim.clientid)
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %     sim.vrep.simxgetlastcmdtime(sim.clientid) -time1
    % % % % % % % % % % % % % end
    % % % % % % % % % % % % % matlab_time = toc
    % % % % % % % % % % % % % aa= sim.getrobot();
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % % % [errorcode]=sim.vrep.simxpausesimulation(sim.clientid,sim.vrep.simx_opmode_oneshot_wait)
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % % robot_init = state([ -0.7999    0.4018    pi/2]);
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % % [a,pos_fl] = sim.vrep.simxgetjointposition(sim.clientid, rollingjoint_fl,sim.vrep.simx_opmode_oneshot_wait);%1-->1
    % % % % % % % % % % % % % [a,pos_fr] = sim.vrep.simxgetjointposition(sim.clientid, rollingjoint_fr, sim.vrep.simx_opmode_oneshot_wait);%4-->2
    % % % % % % % % % % % % % [a,pos_rl] = sim.vrep.simxgetjointposition(sim.clientid, rollingjoint_rl,sim.vrep.simx_opmode_oneshot_wait);%2-->3
    % % % % % % % % % % % % % [a,pos_rr] = sim.vrep.simxgetjointposition(sim.clientid, rollingjoint_rr, sim.vrep.simx_opmode_oneshot_wait);%3-->4
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % % simxgetjointmatrix
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % % [a,pos_fl] = sim.vrep.simxgetobjectposition(sim.clientid, rollingjoint_fl,-1,sim.vrep.simx_opmode_oneshot_wait);%1-->1
    % % % % % % % % % % % % % [a,pos_fr] = sim.vrep.simxgetobjectposition(sim.clientid, rollingjoint_fr,-1, sim.vrep.simx_opmode_oneshot_wait);%4-->2
    % % % % % % % % % % % % % [a,pos_rl] = sim.vrep.simxgetobjectposition(sim.clientid, rollingjoint_rl,-1,sim.vrep.simx_opmode_oneshot_wait);%2-->3
    % % % % % % % % % % % % % [a,pos_rr] = sim.vrep.simxgetobjectposition(sim.clientid, rollingjoint_rr,-1, sim.vrep.simx_opmode_oneshot_wait);%3-->4
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
    % % % % % % % % % % % % % [ errorcode,array_handles,array_intdata,array_floatdata,array_stringdata]=...
    % % % % % % % % % % % % %     simxgetobjectgroupdata(sim.clientid,sim.vrep.sim_appsim_object_type,0,sim.vrep.simx_opmode_oneshot_wait)
    % % % % % % % % % % % % %
    % % % % % % % % % % % % %
