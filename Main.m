clear classes;clear variables;close all;clc;
%load myfilebrkpnts;dbstop(s)
% addpath(genpath('C:\Users\Amirhossein\Desktop\MyWork_TAMU\FIRM_with_simulated_laser\ECMR paper all things\ECMR_paper_codes\completed_after_paper_trends\for ECMR\'))
addpath('./external/rvctools/'); % We need rvctoolbox to run RRT
startup_rvc;
% dbstop if error

% Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.

robot_init = [0 0 0];
% sim = Simulator();
% sim = sim.initialize();
% sim = sim.setRobot(robot_init);

if user_data_class.par.Cancel_Run ~= 1
    % instantiate the simulator
    sim = Simulator();
    sim = sim.initialize();
    
    prob_inst = Planning_Problem(sim);
    prob_inst = prob_inst.solve();
    sim = sim.simDelete();
end