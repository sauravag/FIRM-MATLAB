addpath(genpath('C:\Users\Amirhossein\Documents\GitHub\FIRM-MATLAB\'))

clear classes;clear variables;close all;clc;
seed =502; rand('state',seed);randn('state',seed); %#ok<RAND>
% addpath(genpath('General_functions')); % In this line we add everything inside the "General_functions" to the current paths of the matlab.
startup
load myfilebrkpnts;dbstop(s)
% Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.
if user_data_class.par.Cancel_Run ~= 1
    % instantiate the simulator
    
    robot_init = state.sample_a_valid_state();
    robot_goal = state.sample_a_valid_state();
    robot_init.val = [0 0 0]';%state.sample_a_valid_state();
    robot_goal.val = [0 0.1 0]'; %state.sample_a_valid_state();
    
    
       sim = vrep_interface();
    sim = sim.simInitialize();
    sim = sim.SetRobot(robot_init);
%     sim = Simulator();
%     sim = sim.initialize();
    
    OM = ObservationModel_class; % The object OM is only created for "Constant" properties of the "ObservationModel_class" class to be initialized.
    OM = OM.draw();
    
    lnr_pts_inp = [robot_goal.val;zeros(MotionModel_class.ctDim,1)];
    
    ls = Linear_system_class(lnr_pts_inp);
    SLQG_ = LQG_stationary_class(robot_goal.val);
%     sim = sim. setRobot(robot_init);
    
    mm = MotionModel_class();
    traj = MotionModel_class.generate_open_loop_point2point_traj(robot_init,robot_goal);
    for k = 1:length(traj.u)
        sim = sim.evolve(traj.u(:,k));
        sim = sim.refresh();
        pause(0.02)
    end
    
    traj = MotionModel_class.generate_VALID_open_loop_point2point_traj(robot_init,robot_goal);
    
    prob_inst = Planning_Problem(sim);
    prob_inst = prob_inst.solve();
end