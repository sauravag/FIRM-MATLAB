clear classes;clear variables;close all;clc;

% Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.

if user_data_class.par.Cancel_Run ~= 1
    % instantiate the simulator
    sim = Simulator();
    sim = sim.initialize();
    
    prob_inst = Planning_Problem(sim);
    prob_inst = prob_inst.solve();
    sim = sim.simDelete();
end