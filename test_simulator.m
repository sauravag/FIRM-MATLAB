clear classes;clear variables;close all;clc;
%load myfilebrkpnts;dbstop(s)
% addpath(genpath('C:\Users\Amirhossein\Desktop\MyWork_TAMU\FIRM_with_simulated_laser\ECMR paper all things\ECMR_paper_codes\completed_after_paper_trends\for ECMR\'))
addpath('./external/rvctools/'); % We need rvctoolbox to run RRT
startup_rvc;
% dbstop if error

% Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.

robot_init = state([1 0 pi/4]);
robot_final = state([2 0 0]);

sim = Simulator();
sim = sim.initialize();
% sim = sim.setRobot(robot_init);
MM = MotionModel_class;
nominal_traj = MM.generate_open_loop_point2point_traj(robot_init,robot_final); % generates open-loop trajectories between two start and goal states
for i = size(nominal_traj.u,2)
    sim = sim.evolve(nominal_traj.u(:,i));
    pause(MM.dt)
    aa(:,i)= sim.getRobot();
%     xx(:,i) = aa.val;
end
% xx
    sim = sim.simDelete();

% % % if user_data_class.par.Cancel_Run ~= 1
% % %     % instantiate the simulator
% % %     sim = Simulator();
% % %     sim = sim.initialize();
% % %     
% % %     prob_inst = Planning_Problem(sim);
% % %     prob_inst = prob_inst.solve();
% % %     sim = sim.simDelete();
% % % end