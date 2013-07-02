
%% Commands to get the data of the Laser Scanner
close all;
clear all;clc
robot_init = [0 0 0];
sim = vrep_interface();
sim = sim.simInitialize();
sim = sim.SetRobot(robot_init);

for i = 1:10
   
    sim = sim.getSensorData();
    
end
    
 sim = sim.delete();