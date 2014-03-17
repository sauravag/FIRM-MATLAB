clear classes;clear variables;close all;clc;


% profile on

addpath(genpath('c:\users\amirhossein\desktop\mywork_tamu\firm_with_simulated_laser\ecmr paper all things\ecmr_paper_codes\completed_after_paper_trends\for ecmr\'))
addpath('./external/rvctools/'); % we need rvctoolbox to run rrt
addpath(genpath('C:\Users\Amirhossein\Desktop\MyWork_TAMU\FIRM_with_simulated_laser\ECMR paper all things\ECMR_paper_codes\completed_after_paper_trends\for ECMR\'))

% Add FIRM toolbox and external toolboxes as needed to the Matlab path
addpath(genpath(pwd))
add_external_toolboxes()
% dbstop if error
% Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.
if user_data_class.par.Cancel_Run == 1
    disp('User canceled the program run.')
    break
end

% instantiate the simulator
sim = Simulator();
sim = sim.initialize();
% profview

startNode = state([0,0,0]);
sim = sim.setRobot(startNode);

% This is where you should write your specific planning problem
prob_inst = Planning_Problem(sim);
prob_inst = prob_inst.solve(sim);

% Close the simulator
sim = sim.simDelete();
