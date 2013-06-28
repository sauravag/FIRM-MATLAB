clear classes;clear variables;close all;clc;
seed =502; rand('state',seed);randn('state',seed); %#ok<RAND>
addpath(genpath('General_functions')); % In this line we add everything inside the "General_functions" to the current paths of the matlab.

load myfilebrkpnts;
% Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.
if user_data_class.par.Cancel_Run ~= 1
    % instantiate the simulator
    %     sim = Simulator();
    %     sim = sim.initialize();
    robot_init = state.sample_a_valid_state();
    robot_goal = state.sample_a_valid_state();
    
    sim = vrep_interface();
    sim = sim.simInitialize();
    sim = sim.SetRobot(robot_init);
    %     testRobot = state
    
    
    
    x_next = robot_init.val;
    mm = MotionModel_class();
    traj = MotionModel_class.generate_open_loop_point2point_traj(robot_init,robot_goal);
    for k = 1:length(traj.u)
        sim.evolve(traj.u(:,k));
        w =  zeros(MotionModel_class.wDim,1);
        x_next = MotionModel_class.f_discrete(x_next,traj.u(:,k),w);
        %         x_next =MotionModel_class.f_discrete(x_next); %%robot_init.evolve(traj.u(:,k))
        pos(k) = sim.getRobot()
        x_next
    end
    
    sim.delete();
    %     traj = MotionModel_class.generate_VALID_open_loop_point2point_traj(robot_init,robot_goal);
    
    % % % % %     prob_inst = Planning_Problem(sim);
    % % % % %     prob_inst = prob_inst.solve();
end