
%% Example to use the various sontrol modes of vrep_interface
close all;
clear all;clc

robot_init = [0 0 0];
sim = vrep_interface_youbot();
sim = sim.simInitialize();
sim = sim.setRobot(robot_init);

fprintf('Starting simulation\n');

for i = 1:20
    i
    sim = sim.getSensorData;
    laserData = squeeze(sim.sensor.laserData);
    length(laserData)
    sim = sim.setRobot([0,i/20,0]);
%     idx = find(abs(laserData(1,:))<10 & abs(laserData(2,:))<10);
%     laserData = laserData(:,idx);
%     plot(laserData(1,:),laserData(2,:),'.r')
%     sim = sim.evolve([0.2 0]); %% Input linear velocity in m/s & angular velocity in rad/s
end

sim = sim.delete();





