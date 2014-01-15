clear classes;clear variables;close all;clc;

%% Add FIRM toolbox to the Matlab path
addpath(genpath('C:\Ali\Dropbox\GIT_FIRM\FIRM-MATLAB'))
add_external_toolboxes()

%% Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.
if user_data_class.par.Cancel_Run == 1
    disp('User canceled the program run.')
    break
end

%% instantiate the simulator
sim = Simulator();
sim = sim.initialize();

%% This is where you should write your specific planning problem
% sample two states
startNode = state.sample_a_valid_state();
startNode = startNode.draw();
targetNode = state.sample_a_valid_state();
targetNode = targetNode.draw();
% start Belief
startBelief = belief(startNode.val, eye(state.dim));
% design stabilizer for the target node
controller = SLQG_class(targetNode);

stab_b = Point_stabilizer_SLQG_class(targetNode);
 [next_belief, lost, YesNo_unsuccessful, landed_node_ind, sim] = stab_b.execute(startBelief, 0, sim, 0);

prob_inst = Planning_Problem(sim);
prob_inst = prob_inst.solve();

%% Close the simulator
sim = sim.simDelete();