clear classes;clear variables;close all;clc;

% Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.

if user_data_class.par.Cancel_Run ~= 1
    % instantiate the simulator
robot_init = [0 0 0];
sim = Simulator();
sim = sim.initialize();
sim = sim.setRobot(robot_init);
    

for i = 1:30
    i
    sim = sim.getSensorData;
    laserData = squeeze(sim.sensor.laserData);
%     length(laserData);
%     sim.simTime
%     sim.matTime
%     sim.timeDiff
%     sim = sim.setRobot([0,i/20,0]);
    sim = sim.evolve([1 1 1 1]');

%     idx = find(abs(laserData(1,:))<10 & abs(laserData(2,:))<10);
%     laserData = laserData(:,idx);
%     plot(laserData(1,:),laserData(2,:),'.r')
%     sim = sim.evolve([0.2 0]); %% Input linear velocity in m/s & angular velocity in rad/s
end

%     prob_inst = Planning_Problem(sim);
%     prob_inst = prob_inst.solve();
    sim = sim.simDelete();
end


% %% Example to use the various sontrol modes of vrep_interface
% close all;
% clear all;clc
% 
% robot_init = [0 0 0];
% sim = Simulator();
% sim = sim.simInitialize();
% sim = sim.setRobot(robot_init);
% 
% fprintf('Starting simulation\n');
% 
% for i = 1:20
%     i
%     sim = sim.getSensorData;
%     laserData = squeeze(sim.sensor.laserData);
%     length(laserData)
%     sim = sim.setRobot([0,i/20,0]);
% %     idx = find(abs(laserData(1,:))<10 & abs(laserData(2,:))<10);
% %     laserData = laserData(:,idx);
% %     plot(laserData(1,:),laserData(2,:),'.r')
% %     sim = sim.evolve([0.2 0]); %% Input linear velocity in m/s & angular velocity in rad/s
% end
