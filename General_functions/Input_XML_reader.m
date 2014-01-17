function par_new = Input_XML_reader(old_par, par_new_from_GUI)

%  Parameters (they have to go into an XML)
%=======================================================================================
par_new = par_new_from_GUI;   % first we copy the newly provided parameters from GUI

%=========== Simulator Parameters
if strcmpi(par_new.selected_simulator,'Embedded Simulator')
    typeDef('EmbeddedSimulator' , 'Simulator')
elseif strcmpi(par_new.selected_simulator,'V-Rep Simulator')
    typeDef('VRepSimulator' , 'Simulator')
end

monitor_pos = get(0,'MonitorPositions'); % first line is for the first monitor and second line is for the second monitor.
if prod(monitor_pos(1,:))>0 % Checks if all the elements of the monitor screen coordinates are positive. If not, it means that first monitor should not be used.
    main_monitor_pos = monitor_pos(1,:);
else
    main_monitor_pos = monitor_pos(end,:);
end
x_offset = 35;y_offset = 55;
ratio = 0.8;
par_new.sim.figure_position = [main_monitor_pos(1)+x_offset  ,  main_monitor_pos(2)+y_offset  ,  main_monitor_pos(3)*ratio  ,  main_monitor_pos(4)*ratio]; % if this variable is empty, figure size will be the default value.
par_new.sim.video = 0;
par_new.sim.video_quality = 100;
par_new.sim.interactive_disturbance_allowed = 0 ;
par_new.sim.draw_at_every_n_steps = 4;
par_new.sim.FrameRate = 5;
par_new.sim.env_limits = [0 100 -5 100]; %[-3.75 , 100 , -23.75 , 80]; %[-3 155 -3 155]; %[-10 10 -10 10];%[-6 104 -28 85];%[-5 265 -5 225];%[-6 104 -28 85];
par_new.sim.env_z_limits = [5 20];
par_new.sim.top_obstacle_height_3D = 25;
par_new.sim.bottom_obstacle_height_3D = 0;
par_new.sim.env_background_image_address = 'none'; %'C:\Ali\Academics\PhD_Paper_tryings\Needle_steering\Needle_pics_web\liver.png';%'none'; %'C:\Users\Ali\Desktop\Needle_pics_web\liver-panel5.png';  % This field has to be the address of some image or has to be 'none'
par_new.sim.Lighting_and_3D_plots = 1;
par_new.sim.imageResizeRatio = 0.25;
par_new.sim.viewAngle = [30,40];
par_new.sim.initialZoomRatio = 1.2;%2.5;
par_new.sim.verboseFlag = 1; % (0: suppresses the inermediate code messages intended for debugging purposes | 1: simulator will display messages  )

%=========== Motion Model Parameters
if strcmpi(par_new.selected_motion_model,'Multi RandomWalk robots')
    typeDef('multi_robot_positional_state' , 'state')
    typeDef('multi_robot_positional_belief' , 'belief')
    typeDef('RandomWalk' , 'MotionModel_class')
elseif strcmpi(par_new.selected_motion_model,'Omni-directional three wheel robot')
    typeDef('planar_robot_XYTheta_state' , 'state')
    typeDef('planar_robot_XYTheta_belief' , 'belief')
    typeDef('Omni_directional_robot' , 'MotionModel_class')
elseif strcmpi(par_new.selected_motion_model,'Unicycle')
    typeDef('planar_robot_XYTheta_state' , 'state')
    typeDef('planar_robot_XYTheta_belief' , 'belief')
    typeDef('Unicycle_robot' , 'MotionModel_class')
elseif strcmpi(par_new.selected_motion_model,'Multi Omni-directional robots')
    addpath(genpath('All_state_classes/Multi_planar_robots_XYTheta')); % In this line we import everything inside the "All_motion_models/Omni_directional_robot" directory.
    addpath(genpath('All_belief_classes/Multi_planar_robots_XYTheta')); % In this line we import everything inside the "All_motion_models/Omni_directional_robot" directory.
    addpath(genpath('All_motion_models/Multi_Omni_directional_robots')); % In this line we import everything inside the "All_motion_models/Omni_directional_robot" directory.
elseif strcmpi(par_new.selected_motion_model,'Revolute joint 8arm manipulator')
    typeDef('revolute_joint_manipulator_state' , 'state')
    typeDef('revolute_joint_manipulator_belief' , 'belief')
    typeDef('Revolute_joint_manipulator' , 'MotionModel_class')
elseif strcmpi(par_new.selected_motion_model,'Dynamical planar 8arm manipulator')
    typeDef('Planar_dyn_manipulator_state' , 'state')
    typeDef('Planar_dyn_manipulator_belief' , 'belief')
    typeDef('Dynamical_planar_manipulator' , 'MotionModel_class')
elseif strcmpi(par_new.selected_motion_model,'FixedWing Aircraft')
    typeDef('Six_DOF_robot_state' , 'state')
    typeDef('Six_DOF_robot_belief' , 'belief')
    typeDef('Aircraft_Kinematic' , 'MotionModel_class')
elseif strcmpi(par_new.selected_motion_model,'Kuka YouBot Base')
    typeDef('planar_robot_XYTheta_state' , 'state')
    typeDef('planar_robot_XYTheta_belief' , 'belief')
    typeDef('youbot_base' , 'MotionModel_class')
elseif strcmpi(par_new.selected_motion_model,'Quadrotor')
    typeDef('Quadrotor_state' , 'state')
    typeDef('Quadrotor_belief' , 'belief')
    typeDef('Quadrotor_MM' , 'MotionModel_class')
end

[par_new.motion_model_parameters , par_new.state_parameters] = gather_state_and_motion_model_parameters(old_par, par_new.selected_motion_model);

%=========== Observation Model Parameters
if strcmpi(par_new.selected_observation_model,'Three robot good-poor GPS no comm')
        typeDef('Three_robot_good_poor_GPS_no_comm','ObservationModel_class')
elseif strcmpi(par_new.selected_observation_model,'Three robot good-poor GPS with comm')
        typeDef('Three_robot_good_poor_GPS_with_comm','ObservationModel_class')
elseif strcmpi(par_new.selected_observation_model,'Landmark (range and bearing) sensing')
    typeDef('Landmarks_Range_bearing' , 'ObservationModel_class')
elseif strcmpi(par_new.selected_observation_model,'Multi Robot Sensing')
    addpath(genpath('All_observation_models/Multi_robot_sensing')); % In this line we import everything inside the "All_observation_models/multi_robot_sensing" directory.
elseif strcmpi(par_new.selected_observation_model,'Two robots with no communication')
    typeDef('Two_robots_no_communication','ObservationModel_class')
    %addpath(genpath('All_observation_models/Two_robots_no_communication')); % In this line we import everything inside the "All_observation_models/multi_robot_sensing" directory.
elseif strcmpi(par_new.selected_observation_model,'Two robots with communication')
    addpath(genpath('All_observation_models/Two_robots_with_communication')); % In this line we import everything inside the "All_observation_models/multi_robot_sensing" directory.
elseif strcmpi(par_new.selected_observation_model,'n-arm Manipulator')
    typeDef('Planar_manipulator_wall_sensing_deadzone','ObservationModel_class');
elseif strcmpi(par_new.selected_observation_model,'Dynamical n-arm Manipulator')
    typeDef('Dyn_manipulator_wall_sensing_deadzone','ObservationModel_class');
elseif strcmpi(par_new.selected_observation_model,'3D Landmark (range and bearing)')
    typeDef('Landmarks_3D_Range_bearing','ObservationModel_class');
elseif strcmpi(par_new.selected_observation_model,'Full state information, additive Gaussian noise')    
    typeDef('Full_state_additive_Gaussian','ObservationModel_class');
end
% typeDef('Landmarks_3D_Range_bearing','ObservationModel_class');
par_new.observation_model_parameters = gather_observation_model_parameters(old_par, par_new.observation_model_parameters, par_new.selected_observation_model);


%=========== Planning Problem (Solver) Parameters
if strcmpi(par_new.planning_problem_param.solver, 'Stationary LQG-based FIRM')
    typeDef('Point_PRM_class', 'PRM_class')
    typeDef('Point_stabilizer_SLQG_class', 'stabilizer_class')
elseif strcmpi(par_new.planning_problem_param.solver, 'Periodic LQG-based FIRM')
    typeDef('Orbit_PRM_class', 'PRM_class')
    typeDef('Orbit_stabilizer_PLQG_class', 'stabilizer_class')
elseif strcmpi(par_new.planning_problem_param.solver, 'DFL-and-SKF-based FIRM')
    typeDef('Point_PRM_class', 'PRM_class')
    typeDef('Point_stabilizer_DFL_SKF', 'stabilizer_class')
else
    error('not implemented yet')
end

%=========== Random seed
seed = 502; 
rand('state',seed); %#ok<RAND>
randn('state',seed); %#ok<RAND>
par_new.seed = seed;

%=========== FIRM Node Parameters
mean_neighb_magnifying_coeff = 0; % this coefficient enlarges the mean neighborhood in FIRM node definition, which leads to faster stop and convergence times, if it is greater than 1.
cov_neighb_magnifying_coeff = 0; % this coefficient enlarges the covariance neighborhood in FIRM node definition, which leads to faster stop and convergence times, if it is greater than 1.
if strcmpi(par_new.selected_motion_model,'Multi RandomWalk robots')
    n = par_new.state_parameters.num_robots;
    tmp_vector = repmat( [0.08 ; 0.08 ] , n , 1);
    par_new.FIRM_node_parameters.mean_neighborhood_size = tmp_vector*mean_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian.
    par_new.FIRM_node_parameters.cov_neighborhood_size = tmp_vector*tmp_vector'*cov_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian. % This is a matrix.
    % Hbliefe convergece-related parameters:
    GHb_conv_reg_thresh = tmp_vector*35; % distance threshold for either Xg_mean or Xest_mean_mean; Xdist has to be a column vector
elseif strcmpi(par_new.selected_motion_model,'Multi Omni-directional robots')
    n = par_new.state_parameters.num_robots;
    tmp_vector = repmat( [0.08 ; 0.08 ; 3 *pi/180 ] , n , 1);
    par_new.FIRM_node_parameters.mean_neighborhood_size = tmp_vector*mean_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian.
    par_new.FIRM_node_parameters.cov_neighborhood_size = tmp_vector*tmp_vector'*cov_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian. % This is a matrix.
    % Hbliefe convergece-related parameters:
    GHb_conv_reg_thresh = tmp_vector*35; % distance threshold for either Xg_mean or Xest_mean_mean; Xdist has to be a column vector
elseif strcmpi(par_new.selected_motion_model,'Revolute joint 8arm manipulator')
    tmp_vector = ones(par_new.state_parameters.stateDim , 1)*5*pi/180;
    par_new.FIRM_node_parameters.mean_neighborhood_size = tmp_vector*mean_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian.
    par_new.FIRM_node_parameters.cov_neighborhood_size = tmp_vector*tmp_vector'*cov_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian. % This is a matrix.
    % Hbliefe convergece-related parameters:
    GHb_conv_reg_thresh = tmp_vector*35; % distance threshold for either Xg_mean or Xest_mean_mean; Xdist has to be a column vector
elseif strcmpi(par_new.selected_motion_model,'Dynamical planar 8arm manipulator')
    tmp_vector = [ones(par_new.state_parameters.stateDim/2 , 1)*5*pi/180;ones(par_new.state_parameters.stateDim/2 , 1)*10*pi/180];
    par_new.FIRM_node_parameters.mean_neighborhood_size = tmp_vector*mean_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian.
    par_new.FIRM_node_parameters.cov_neighborhood_size = tmp_vector*tmp_vector'*cov_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian. % This is a matrix.
    % Hbliefe convergece-related parameters:
    GHb_conv_reg_thresh = tmp_vector*35; % distance threshold for either Xg_mean or Xest_mean_mean; Xdist has to be a column vector
elseif strcmpi(par_new.selected_motion_model,'FixedWing Aircraft')
    tmp_vector = repmat( [0.1 ; 0.1 ; 0.1; 0.1 ; 0.1 ; 0.1 ; 0.1] , 1 , 1);
    par_new.FIRM_node_parameters.mean_neighborhood_size = tmp_vector*mean_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian.
    par_new.FIRM_node_parameters.cov_neighborhood_size = tmp_vector*tmp_vector'*cov_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian. % This is a matrix.
    % Hbliefe convergece-related parameters:
    GHb_conv_reg_thresh = tmp_vector*35; % distance threshold for either Xg_mean or Xest_mean_mean; Xdist has to be a column vector
elseif strcmpi(par_new.selected_motion_model,'Quadrotor')
    tmp_vector = [1 ; 1 ; 1; 20*pi/180 ; 20*pi/180 ; 20*pi/180 ; inf ; inf ; inf ; inf ; inf ; inf];
    par_new.FIRM_node_parameters.mean_neighborhood_size = tmp_vector*mean_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian.
    par_new.FIRM_node_parameters.cov_neighborhood_size = tmp_vector*tmp_vector'*cov_neighb_magnifying_coeff ; % note that the last entry, ie theta's neighborhood, has to be in radian. % This is a matrix.
    % Hbliefe convergece-related parameters:
    GHb_conv_reg_thresh = tmp_vector*35; % distance threshold for either Xg_mean or Xest_mean_mean; Xdist has to be a column vector
else
    par_new.FIRM_node_parameters.mean_neighborhood_size = [0.08 ; 0.08 ; 3 *pi/180 ]*mean_neighb_magnifying_coeff ; % this only works for 3D state spaces % note that the last entry, ie theta's neighborhood, has to be in radian.
    par_new.FIRM_node_parameters.cov_neighborhood_size = [0.08 ; 0.08 ; 3 *pi/180 ]*[0.08 ; 0.08 ; 3 *pi/180 ]'*cov_neighb_magnifying_coeff ; % this only works for 3D state spaces % note that the last entry, ie theta's neighborhood, has to be in radian. % This is a matrix.
    % Hbliefe convergece-related parameters:
    GHb_conv_reg_thresh = [0.08 ; 0.08 ; 1.5*pi/180]*35; % distance threshold for either Xg_mean or Xest_mean_mean; Xdist has to be a column vector
end

BigX_thresh = [GHb_conv_reg_thresh;GHb_conv_reg_thresh];
par_new.FIRM_node_parameters.GHb_conv_BigX_thresh = BigX_thresh; % distance threshold for both Xg_mean and Xest_mean_mean in the single vector
par_new.FIRM_node_parameters.GHb_conv_Pest_thresh = GHb_conv_reg_thresh*GHb_conv_reg_thresh'; % defines the convergence threshold for Pest
par_new.FIRM_node_parameters.GHb_conv_BigCov_thresh = BigX_thresh*BigX_thresh'; % defines the convergence threshold for BigCov

%=========== Stabilizer Parameters
par_new.stabilizer_parameters.max_stopping_time = 50;
par_new.stabilizer_parameters.draw_cov_centered_on_nominal = 0;

%=========== MonteCarlo Simulation
par_new.par_n = 2; % number of particles
par_new.cost_gain = 10;

%=========== (LQR design) Node and Edge controller
LQR_cost_coef=[0.03*0.1 , 0.03*0.1 , 0.1];  % first entry is the "final state cost coeff". The second is the "state cost coeff", and the third is the "control cost coeff".
par_new.Final_state_cost=eye(par_new.state_parameters.stateDim)*LQR_cost_coef(1);
par_new.state_cost=eye(par_new.state_parameters.stateDim)*LQR_cost_coef(2);
par_new.control_cost=eye(par_new.motion_model_parameters.controlDim)*LQR_cost_coef(3);
if strcmpi(par_new.selected_motion_model,'Multi RandomWalk robots')
    n = par_new.state_parameters.num_robots;
    par_new.valid_linearization_domain = repmat([3;3]*4 , n , 1);
elseif strcmpi(par_new.selected_motion_model,'Multi Omni-directional robots')
    n = par_new.state_parameters.num_robots;
    par_new.valid_linearization_domain = repmat([3;3;75*pi/180]*3 , n , 1);
elseif strcmpi(par_new.selected_motion_model,'Revolute joint 8arm manipulator')
    par_new.valid_linearization_domain = ones(par_new.state_parameters.stateDim , 1)*75*pi/180;
elseif strcmpi(par_new.selected_motion_model,'Dynamical planar 8arm manipulator')
    par_new.valid_linearization_domain = [ones(par_new.state_parameters.stateDim/2 , 1)*75*pi/180; ones(par_new.state_parameters.stateDim/2 , 1)*1000*pi/180];
elseif strcmpi(par_new.selected_motion_model, 'FixedWing Aircraft')
    par_new.valid_linearization_domain = [3;3;3;1;1;1;1]*3;
elseif strcmpi(par_new.selected_motion_model,'Quadrotor')
    par_new.valid_linearization_domain = [3;3;3;75*pi/180;75*pi/180;75*pi/180;inf;inf;inf;inf;inf;inf];
else
    par_new.valid_linearization_domain = [3;3;75*pi/180]*3;
end

par_new.state_cost_ratio_for_stationary_case = 5; % note that "state_cost" is for the trajectory tracking. Usually in point stabilization, we need more force on state and less on control. So, we multiply the "state_cost" to an appropriate ratio, i.e., "state_cost_ratio_for_stationary_case". Note that this ratio has to be greater than 1.
par_new.control_cost_ratio_for_stationary_case = 1/5; % note that "control_cost" is for the trajectory tracking. Usually in point stabilization, we need more force on state and less on control. So, we multiply the "control_cost" to an appropriate ratio, i.e., "control_cost_ratio_for_stationary_case". Note that this ratio has to be LESS than 1.

%=========== HBRM cost
par_new.alpha_for_HBRM_cost = [0.01,0.1,1]; % respectively, corresponding to "stopping_time", "success probability", and "filtering_cost".

%=========== Roadmap Type and Construction
par_new.RoadMap = 'FIRM'; % This parameter can be HBRM or FIRM
par_new.No_history = 1;
par_new.No_plot = 1; % this is for plots in construction phase. The execution phase plots are different.

%=========== PRM parameters

par_new.PRM_parameters.neighboring_distance_threshold = 30; %* 1.25 * 1000;% * 0.3;
par_new.PRM_parameters.PRM_node_text = 1; % if this is one, the number of nodes will be written on the figure.
par_new.PRM_parameters.PRM_node_plot_properties =  {'RobotShape','triangle','robotSize',0.8};% {'RobotShape','triangle','robotSize',2};
par_new.PRM_parameters.draw_edges_flag = 1;

% =========== Orbit parameters
% par_new.PRM_parameters.orbit_text_size = 12;  % Default value for "OrbitTextSize" property.
% par_new.PRM_parameters.orbit_text_shift = 0.8; % for some reason MATLAB shifts the starting point of the text a little bit to the right. So, we return it back by this amount.
% par_new.PRM_parameters.orbit_text_color = 'b'; % Default value for "OrbitTextColor" property.
% par_new.PRM_parameters.orbit_robot_shape = 'triangle'; % The shape of robot (to draw trajectories and to show direction of edges and orbits)
% par_new.PRM_parameters.orbit_robot_size = 1; % Robot size on orbits (to draw trajectories and to show direction of edges and orbits)
par_new.PRM_parameters.node_to_orbit_trajectories_flag = 1; % Make it one if you want to see the node-to-orbit trajectories. Zero, otherwise.
% par_new.PRM_parameters.orbit_color = 'k'; % User-provided value for "orbit_color" property.
% par_new.PRM_parameters.orbit_width = 2; % User-provided value for "orbit_width" property.
% par_new.PRM_parameters.orbit_trajectory_flag = 0; % Make it one if you want to see the orbit trajectories. Zero, otherwise.
% par_new.PRM_parameters.edge_spec = '-b'; % edge line color and type
% par_new.PRM_parameters.edge_width = 2; % edge line width
par_new.PRM_parameters.num_nodes_on_orbits = 3; % number of nodes on each orbit
% par_new.PRM_parameters.orbit_length = 50; % the length of orbit (orbit's time period)
% par_new.PRM_parameters.orbit_radius = 4;




%===========  Dynamic Programming parameters
par_new.initial_values = 100;
par_new.initial_value_goal = 500;
par_new.failure_cost_to_go = 15;
par_new.selected_nodes_for_plotting_feedback_pi = [];%setdiff(1:22, [4,7,19,17,8,20,12,3,6,21]);
par_new.DP_convergence_threshold = 1e-2;

%===========  Replanning
par_new.replanning = 0;
par_new.goBack_to_nearest_node = 0; % this does not work correctly, yet.

%=======================================================================================
%=======================================================================================
% End of Parameters section!
end

function [motion_model_parameters , state_parameters] = gather_state_and_motion_model_parameters(old_par, selected_motion_model)
% --- This function returns the parameters needed in the selected motion model.
motion_model_parameters = old_par.motion_model_parameters;  % Here, we load the "old motion model parameters", so that the old information that we do not change, remains unaffected.
if strcmpi(selected_motion_model,'Omni-directional three wheel robot')
    state_parameters.stateDim = 3;
    state_parameters.sup_norm_weights_nonNormalized = 1./[1 ; 1 ; inf]; % You can think of the right-most vector (in the denominator) as the ractangular neighborhood used in finding neighbor nodes in constructing PRM graph. Note that this must be a column vector.
    motion_model_parameters.controlDim=3;
    motion_model_parameters.robot_link_length = 0.2; %str2double(get(handles.edit_omni_link_length,'String'));
    motion_model_parameters.dt = 0.1;
    motion_model_parameters.V_const_path=5; % nominal linear velocity
    motion_model_parameters.omega_const_path=90*pi/180; % nominal angular velocity % note that this is the turning angle in one second. So, it will be multiplied by "dt" to return the turning angle in "dt".
    motion_model_parameters.eta_u_omni = [0; 0; 0];  %str2num(get(handles.edit_eta_u_omni,'String'))'; %#ok<ST2NM> % note that eta_u in this case is a three by one vector, reprensing eta for velocity of each of omni-dir wheels.
    motion_model_parameters.sigma_b_u_omni = [0; 0; 0];  % note that sigma_b_u in this case is a three by one vector, reprensing sigma_b (bias variance) for linear velocity and angular velocity.
    P_rootsqaure_Wg_diags=[0.2 ; 0.2 ; 4*pi/180]*2;
    motion_model_parameters.P_Wg=diag(P_rootsqaure_Wg_diags.^2);
elseif strcmpi(selected_motion_model,'Multi Omni-directional robots')
    n = 2;
    state_parameters.num_robots=n;
    state_parameters.stateDim = 3*n;
    state_parameters.sup_norm_weights_nonNormalized = repmat(1./[1 ; 1 ; inf] , n , 1); % You can think of the right-most vector (in the denominator) as the ractangular neighborhood used in finding neighbor nodes in constructing PRM graph. Note that this must be a column vector.
    motion_model_parameters.controlDim=3*n;
    motion_model_parameters.robot_link_length = 0.2; %str2double(get(handles.edit_omni_link_length,'String'));
    motion_model_parameters.dt = 0.1;
    motion_model_parameters.V_const_path_team=ones(1,n); % nominal linear velocity
    motion_model_parameters.omega_const_path_team=ones(1,n)*90*pi/180; % nominal angular velocity % note that this is the turning angle in one second. So, it will be multiplied by "dt" to return the turning angle in "dt".
    motion_model_parameters.eta_u_omni_team = zeros(3*n,1); % %str2num(get(handles.edit_eta_u_omni,'String'))'; %#ok<ST2NM> % note that eta_u in this case is a three by one vector, reprensing eta for velocity of each of omni-dir wheels.
    motion_model_parameters.sigma_b_u_omni_team = zeros(3*n,1); % % note that sigma_b_u in this case is a three by one vector, reprensing sigma_b (bias variance) for linear velocity and angular velocity.
    P_rootsqaure_Wg_diags_team = repmat( [0.2 ; 0.2 ; 4*pi/180]*2, n ,1 ); % this is just a vector
    motion_model_parameters.P_Wg_team = diag(P_rootsqaure_Wg_diags_team.^2);
elseif strcmpi(selected_motion_model,'Multi RandomWalk robots')
    n = 3;
    state_parameters.num_robots=n;
    state_parameters.stateDim = 2*n;
    state_parameters.sup_norm_weights_nonNormalized = repmat(1./[1 ; 1 ] , n , 1); % You can think of the right-most vector (in the denominator) as the ractangular neighborhood used in finding neighbor nodes in constructing PRM graph. Note that this must be a column vector.
    motion_model_parameters.controlDim=2*n;
    motion_model_parameters.dt = 0.1;
    motion_model_parameters.V_const_path_team=ones(2*n,1)*20; % nominal linear velocity
    P_rootsqaure_Wg_diags_team = repmat( [0.2 ; 0.2]*2, n ,1 ); % this is just a vector
    motion_model_parameters.P_Wg_team = diag(P_rootsqaure_Wg_diags_team.^2);    
elseif strcmpi(selected_motion_model,'Unicycle')
    state_parameters.stateDim = 3;
    state_parameters.sup_norm_weights_nonNormalized = 1./[1 ; 1 ; inf]; % You can think of the right-most vector (in the denominator) as the ractangular neighborhood used in finding neighbor nodes in constructing PRM graph. Note that this must be a column vector.
    motion_model_parameters.controlDim=2;
    motion_model_parameters.base_length = 13; %str2double(get(handles.edit_unicycle_base_length,'String'));
    motion_model_parameters.dt = 0.1;
    motion_model_parameters.V_const_path = 4; % nominal linear velocity
    motion_model_parameters.omega_const_path=25*pi/180; % nominal angular velocity
    motion_model_parameters.eta_u_unicycle = [0; 0];  % %str2num(get(handles.edit_eta_u_unicycle,'String'))'; %#ok<ST2NM> % note that eta_u in this case is a two by one vector, reprensing eta for linear velocity and angular velocity.
    motion_model_parameters.sigma_b_u_unicycle = [0; 0]; % % note that sigma_b_u in this case is a two by one vector, reprensing sigma_b (bias variance) for linear velocity and angular velocity.
    P_rootsqaure_Wg_diags=[0.2 ; 0.2 ; 4*pi/180];
    motion_model_parameters.P_Wg=diag(P_rootsqaure_Wg_diags.^2);
elseif strcmpi(selected_motion_model,'Revolute joint 8arm manipulator')
    n = 8;
    state_parameters.num_revolute_joints = n;
    state_parameters.stateDim = n;
    state_parameters.sup_norm_weights_nonNormalized = ones(n , 1); % You can think of the right-most vector (in the denominator) as the ractangular neighborhood used in finding neighbor nodes in constructing PRM graph. Note that this must be a column vector.
    motion_model_parameters.controlDim=n;
elseif strcmpi(selected_motion_model,'Dynamical planar 8arm manipulator')
    n = 16;
    state_parameters.num_revolute_joints = n/2;
    state_parameters.stateDim = n;
    state_parameters.sup_norm_weights_nonNormalized = ones(n , 1); % You can think of the right-most vector (in the denominator) as the ractangular neighborhood used in finding neighbor nodes in constructing PRM graph. Note that this must be a column vector.
    motion_model_parameters.controlDim = n/2;
elseif strcmpi(selected_motion_model,'FixedWing Aircraft')
    state_parameters.stateDim = 7;
    state_parameters.sup_norm_weights_nonNormalized = ones(state_parameters.stateDim , 1); 
    disp('state norm for aircraft model needs to be fixed')
    motion_model_parameters.controlDim = 4;
    motion_model_parameters.dt = 0.1;
    motion_model_parameters.eta_u_aircraft = [0.005;0.005;0.005;0.005];%[0.01 ; deg2rad(0.025) ; deg2rad(0.025) ; deg2rad(0.025)];  
    motion_model_parameters.sigma_b_u_aircraft = [0.02; deg2rad(0.25);deg2rad(0.25); deg2rad(0.25)];%[0.01 ; deg2rad(0.2); deg2rad(0.2); deg2rad(0.2)];  
    P_rootsqaure_Wg_diags = [0.02 ; 0.02 ; 0.02 ; 0.01 ; 0.01 ; 0.01 ; 0.01];
    motion_model_parameters.P_Wg = diag(P_rootsqaure_Wg_diags.^2);
elseif strcmpi(selected_motion_model,'Kuka YouBot Base')
    state_parameters.stateDim = 3;
    state_parameters.sup_norm_weights_nonNormalized = 1./[1 ; 1 ; inf]; % You can think of the right-most vector (in the denominator) as the ractangular neighborhood used in finding neighbor nodes in constructing PRM graph. Note that this must be a column vector.
    motion_model_parameters.controlDim = 4;
    motion_model_parameters.dt = 0.1;
    motion_model_parameters.eta_u_KukaBase = [0; 0; 0; 0]; 
    motion_model_parameters.sigma_b_u_KukaBase = [0; 0; 0; 0];  
    P_rootsqaure_Wg_diags=[0.2 ; 0.2 ; 4*pi/180]*2;
    motion_model_parameters.P_Wg=diag(P_rootsqaure_Wg_diags.^2);
    motion_model_parameters.distBetweenFrontWheels = 0.158*2; % from YouBot datasheet
    motion_model_parameters.distBetweenFrontAndBackWheels = 0.228*2; % from YouBot datasheet
elseif strcmpi(selected_motion_model,'Quadrotor')
    state_parameters.stateDim = 12;
    state_parameters.sup_norm_weights_nonNormalized = 1./[1 ; 1 ; 1; inf(9,1)]; % You can think of the right-most vector (in the denominator) as the ractangular neighborhood used in finding neighbor nodes in constructing PRM graph. Note that this must be a column vector.
    motion_model_parameters.controlDim = 4;
    motion_model_parameters.dt = 0.1;
    motion_model_parameters.eta_u_quadrotor = [0; 0; 0; 0];  % str2num(get(handles.eta_u_quadrotor,'String'))'; %#ok<ST2NM> % note that eta_u in this case is a four by one vector, reprensing the dependence of control noise on the magnitude of the control vector.
    motion_model_parameters.sigma_b_u_quadrotor = [0; 0; 0; 0];  % note that sigma_b_u in this case is a four by one vector, reprensing sigma_b (bias variance) for the control-independent part of the control noise.
    P_rootsqaure_Wg_diags=[0.2 ; 0.2 ; 0.2 ; 0.001 ; 0.001 ; 0.001; 4*pi/180 ; 4*pi/180 ; 4*pi/180 ; 0.001 ; 0.001 ; 0.001];
    motion_model_parameters.P_Wg=diag(P_rootsqaure_Wg_diags.^2);
else
    error('SFMP algorithm: The selected motion model does not match with the existing database');
end
end

function observation_model_parameters_new = gather_observation_model_parameters(old_par, observation_model_parameters_from_GUI, selected_observation_model)
% --- This fucntion returns the parameters needed in the selected observation model.
observation_model_parameters_old = old_par.observation_model_parameters;  % Here, we load the "old motion model parameters", so that the old information that we do not change, remains unaffected.
observation_model_parameters_new = observation_model_parameters_old; % This line is written only to increase the readability of the code.
observation_model_parameters_new.interactive_OM = observation_model_parameters_from_GUI.interactive_OM; % This line is needed since the user may overwrite the "manual_landmark" property through GUI
observation_model_parameters_new.eta=[0.2100 , 0.2100]/2;
observation_model_parameters_new.sigma_b=[0.2 , 0*pi/180]/2;
end