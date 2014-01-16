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
% startNode = state.sample_a_valid_state();
% startNode = startNode.draw();
startNode = state([0,0,pi/4]);
startNode = startNode.draw();





% targetNode = state.sample_a_valid_state();
% targetNode = targetNode.draw();
targetNode = state([1 1 pi/2]);
targetNode = targetNode.draw();



% start Belief
startBelief = belief(startNode.val, 0.001*eye(state.dim));
% design stabilizer for the target node
controller = SLQG_class(targetNode);

b_init=belief(startNode,eye(state.dim))

sim = sim.setRobot(startNode)
stab_b = Point_stabilizer_SLQG_class(targetNode);
stab_b = construct_reachable_FIRM_nodes(stab_b)
[next_belief, lost, YesNo_unsuccessful, landed_node_ind, sim] = stab_b.execute( b_init, 0, sim, 0)
% [next_belief, lost, YesNo_unsuccessful, landed_node_ind, sim] = execute(obj, current_belief, convergence_time, sim, noiseFlag)
% stab_b.execute(

prob_inst = Planning_Problem(sim);
prob_inst = prob_inst.solve();

%% Close the simulator
sim = sim.simDelete();