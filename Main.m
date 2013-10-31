clear classes;clear variables;close all;clc;
load myfilebrkpnts;dbstop(s)
addpath(genpath('C:\Users\Amirhossein\Desktop\MyWork_TAMU\FIRM_with_simulated_laser\ECMR paper all things\ECMR_paper_codes\completed_after_paper_trends\for ECMR\'))
addpath('./external/rvctools/'); % We need rvctoolbox to run RRT
startup_rvc;


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
    
%     robot_init = state.sample_a_valid_state();
%     robot_goal = state.sample_a_valid_state();
%     
%     sim = sim.setRobot(robot_init);
%        
%     mm = MotionModel_class();
%     traj = MotionModel_class.generate_open_loop_point2point_traj(robot_init,robot_goal);
%     for k = 1:length(traj.u)
%         
%         sim = sim.evolve(traj.u(:,k),0);
%         sim = sim.refresh();
%         pause(0.02)
%     end
    
    %traj = MotionModel_class.generate_VALID_open_loop_point2point_traj(robot_init,robot_goal);
    
    prob_inst = Planning_Problem(sim);
    prob_inst = prob_inst.solve();
    sim = sim.simDelete();
end