
%% Commands to get the data of the Laser Scanner

% % % % % close all;
% % % % % clear all;clc
% % % % % addpath(genpath('C:\Users\Amirhossein\Documents\GitHub\FIRM-MATLAB\'))
% % % % % 
% % % % % clear classes;clear variables;close all;clc;
% % % % % seed =502; rand('state',seed);randn('state',seed); %#ok<RAND>
% % % % % addpath(genpath('General_functions')); % In this line we add everything inside the "General_functions" to the current paths of the matlab.
% % % % % % startup
% % % % % load myfilebrkpnts;dbstop(s)
% % % % % % Parameters
% % % % % user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.
% % % % % addpath(genpath('C:\Users\Amirhossein\Documents\GitHub\FIRM-MATLAB'));
% % % % % addpath(genpath('C:\Users\Amirhossein\Desktop\MyWork_TAMU\FIRM_with_simulated_laser\ECMR paper all things\ECMR_paper_codes\completed_after_paper_trends\for ECMR\'))
robot_init = [0 0 0];
sim = Simulator();
sim = sim.initialize();
% sim = sim.setRobot(robot_init);
thresholds=default_thresholds_func();
sim=sim.evolve([0,0,4]);

for i = 1:20
%     sim=sim.evolve([0,0,4]);
    sim = sim.getSensorData;
    i
    laserData = squeeze(sim.sensor.laserData);
    if ~isempty(laserData )
%     removing outlier point
    idx = find(abs(laserData(1,:))<10 & abs(laserData(2,:))<10);
    if i>3
        scan.x = -laserData(2,idx).*100;
        scan.y = laserData(1,idx).*100;
        new_features_set=hierarchical_feature_extracting(scan,thresholds,'new');
        axis equal
    end
    end
end

sim = sim.simDelete();
 

 
 
 
 