clear sim;
clear classes;clear variables;
close all;clc;
seed =502; rand('state',seed);randn('state',seed); %#ok<RAND>
addpath(genpath('General_functions')); % In this line we add everything inside the "General_functions" to the current paths of the matlab.
cd 'C:\Users\Ajinkya\Dropbox\FIRM_toolbox_ver_current';


load myfilebrkpnts;
% % Parameters
user_data = user_data_class; % The object user_data will never be used. This line only cause the "Constant" properties of the "user_data_class" class to be initialized.
if user_data_class.par.Cancel_Run ~= 1
    % instantiate the simulator
    %     sim = Simulator();
    %     sim = sim.initialize();
    robot_init = state.sample_a_valid_state();
%     robot_init = state([0 0 0]);
    robot_goal = state.sample_a_valid_state();
%     robot_goal = state([5 0 pi/2]);

    sim = vrep_interface();
    sim = sim.simInitialize();
    sim = sim.SetRobot(robot_init);
    
    x_next = robot_init.val;
    mm = MotionModel_class();
    traj = MotionModel_class.generate_open_loop_point2point_traj(robot_init,robot_goal);
    tic
    for k = 1:length(traj.u)
        
        %Vrep computation
        sim = sim.evolve(traj.u(:,k),k);
        robot_ali = sim.getRobot();
        pos = robot_ali.robot_position;
        ori = robot_ali.robot_orientation;
%         [pos(1) pos(2) ori(3)]
        V_rep(k,1) = pos(1);
        V_rep(k,2) = pos(2);

        
        % firm computation
        w =zeros(MotionModel_class.wDim,1);
        x_next = MotionModel_class.f_discrete(x_next,traj.u(:,k),w); %%robot_init.evolve(traj.u(:,k))
        Firm(k,1) = x_next(1);
        Firm(k,2) = x_next(2);
%         x_next

    end
    toc
    sim = sim.delete();
    %     traj = MotionModel_class.generate_VALID_open_loop_point2point_traj(robot_init,robot_goal);
    
    % % % % %     prob_inst = Planning_Problem(sim);
    % % % % %     prob_inst = prob_inst.solve();
end