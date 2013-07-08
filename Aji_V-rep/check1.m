
%% Commands to get the data of the Laser Scanner
close all;
clear all;clc
addpath(genpath('C:\Users\Amirhossein\Documents\GitHub\FIRM-MATLAB'));
addpath(genpath('C:\Users\Amirhossein\Desktop\MyWork_TAMU\FIRM_with_simulated_laser\ECMR paper all things\ECMR_paper_codes\completed_after_paper_trends\for ECMR\'))
robot_init = [0 0 0];
sim = vrep_interface();
sim = sim.simInitialize();
sim = sim.SetRobot(robot_init);
thresholds=default_thresholds_func();

for i = 1:100
    
    sim = sim.getSensorData;
    laserData = squeeze(sim.sensor.laserData);
%     removing outlier point
    idx = find(abs(laserData(1,:))<10 & abs(laserData(2,:))<10);
    if i>3
        scan.x = laserData(1,idx).*100;
        scan.y = laserData(2,idx).*100;
        new_features_set=hierarchical_feature_extracting(scan,thresholds,'new');
    end
end

sim = sim.delete();
 

 
 
 
 