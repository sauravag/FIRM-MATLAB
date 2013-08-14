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
    sim = sim.setRobot(state([4,4,3*pi/2]));
    belief_init = belief(state([4,4,3*pi/2]), eye(state.dim));
    
    sim = sim.setBelief(belief_init);
    sim = sim.refresh();
    
    %     belief_init = belief(state([4,4,3*pi/2]), eye(state.dim));
    robot_goal = state([0 0 0]);
    
    controller = SLQG_class(robot_goal.val);
    
    b = belief_init;
    
    %     for i=1:1000
    
    %     end
    
    noiseFlag = 0;
    for i =1:1:1000
        
        [b, reliable,sim]= controller.executeOneStep(b,sim,noiseFlag);
        sim = sim.setBelief(b);
        sim = sim.refresh();
        
        drawnow
        pause(0.1)
    end
    
    
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