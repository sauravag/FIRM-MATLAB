clear classes;clear variables;close all;clc;
%load myfilebrkpnts;dbstop(s)
% addpath(genpath('C:\Users\Amirhossein\Desktop\MyWork_TAMU\FIRM_with_simulated_laser\ECMR paper all things\ECMR_paper_codes\completed_after_paper_trends\for ECMR\'))
% dbstop if error
a = which(['Landmarks_3D_Range_bearing','.m']);
which Landmarks_3D_Range_bearing.m

% old = 'Landmarks_3D_Range_bearing';

% a = which([old,'.m']);
% Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.

robot_init = state([110 0 pi/4]);
robot_final = state([150 0 0]);

sim = Simulator();
sim = sim.initialize();
sim = sim.setRobot(robot_init);
MM = MotionModel_class;
nominal_traj = MM.generate_open_loop_point2point_traj(robot_init,robot_final); % generates open-loop trajectories between two start and goal states
for i = 1:size(nominal_traj.u,2)
%     sim = sim.setRobot(nominal_traj.x(:,i));
        sim = sim.evolve(nominal_traj.u(:,i));
%     sim = sim.draw()
    sim = sim.refresh()
    %     pause(.1)
    %     aa(:,i)= sim.getRobot();
    %     xx(:,i) = aa.val;
    i
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