clear classes;clear variables;close all;clc;
seed =502; rand('state',seed);randn('state',seed); %#ok<RAND>
addpath(genpath('General_functions')); % In this line we add everything inside the "General_functions" to the current paths of the matlab.
startup
load myfilebrkpnts;dbstop(s)

% Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.
if user_data_class.par.Cancel_Run ~= 1
    % instantiate the simulator
    sim = Simulator();
    sim = sim.initialize();
    
    prob_inst = Planning_Problem(sim);
    prob_inst = prob_inst.solve();
end