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
    sim = Simulator();
    sim = sim.initialize();
    
    robot_goal = state([0 0 0]);
    
    lnr_pts_inp.x = robot_goal.val;
    lnr_pts_inp.u = zeros(MotionModel_class.ctDim,1);
    
    ls = Linear_system_class(lnr_pts_inp);
    controller = LQG_stationary_class(robot_goal.val);
    
    controller.propagate_Hstate
    
    sim = sim. setRobot(robot_init);
    
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