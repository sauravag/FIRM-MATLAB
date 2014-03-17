function reTypeDef(par_new)

%=========== Simulator Parameters
switch par_new.selected_simulator
    case 'Embedded Simulator'
        typeDef('EmbeddedSimulator' , 'Simulator')
    case 'V-Rep Simulator'
        typeDef('VRepSimulator' , 'Simulator')
end


%=========== Motion Model Parameters
switch  par_new.selected_motion_model
    case 'Multi RandomWalk robots'
        typeDef('multi_robot_positional_state' , 'state')
        typeDef('multi_robot_positional_belief' , 'belief')
        typeDef('RandomWalk' , 'MotionModel_class')
    case 'Omni-directional three wheel robot'
        typeDef('planar_robot_XYTheta_state' , 'state')
        typeDef('planar_robot_XYTheta_belief' , 'belief')
        typeDef('Omni_directional_robot' , 'MotionModel_class')
    case 'Unicycle'
        typeDef('planar_robot_XYTheta_state' , 'state')
        typeDef('planar_robot_XYTheta_belief' , 'belief')
        typeDef('Unicycle_robot' , 'MotionModel_class')
    case 'Multi Omni-directional robots'
        addpath(genpath('All_state_classes/Multi_planar_robots_XYTheta')); % In this line we import everything inside the "All_motion_models/Omni_directional_robot" directory.
        addpath(genpath('All_belief_classes/Multi_planar_robots_XYTheta')); % In this line we import everything inside the "All_motion_models/Omni_directional_robot" directory.
        addpath(genpath('All_motion_models/Multi_Omni_directional_robots')); % In this line we import everything inside the "All_motion_models/Omni_directional_robot" directory.
    case 'Revolute joint 8arm manipulator'
        typeDef('revolute_joint_manipulator_state' , 'state')
        typeDef('revolute_joint_manipulator_belief' , 'belief')
        typeDef('Revolute_joint_manipulator' , 'MotionModel_class')
    case 'Dynamical planar 8arm manipulator'
        typeDef('Planar_dyn_manipulator_state' , 'state')
        typeDef('Planar_dyn_manipulator_belief' , 'belief')
        typeDef('Dynamical_planar_manipulator' , 'MotionModel_class')
    case 'FixedWing Aircraft'
        typeDef('Six_DOF_robot_state' , 'state')
        typeDef('Six_DOF_robot_belief' , 'belief')
        typeDef('Aircraft_Kinematic' , 'MotionModel_class')
    case 'Kuka YouBot Base'
        typeDef('planar_robot_XYTheta_state' , 'state')
        typeDef('planar_robot_XYTheta_belief' , 'belief')
        typeDef('youbot_base' , 'MotionModel_class')
    case 'Quadrotor'
        typeDef('Quadrotor_state' , 'state')
        typeDef('Quadrotor_belief' , 'belief')
        typeDef('Quadrotor_MM' , 'MotionModel_class')
end


%=========== Observation Model Parameters
switch par_new.selected_observation_model
    case 'Three robot good-poor GPS no comm'
        typeDef('Three_robot_good_poor_GPS_no_comm','ObservationModel_class')
    case 'Laser Scanner (range bearing)'
        typeDef('Landmarks_Range_bearing_laser','ObservationModel_class')
    case 'Three robot good-poor GPS with comm'
        typeDef('Three_robot_good_poor_GPS_with_comm','ObservationModel_class')
    case 'Landmark (range and bearing) sensing'
        typeDef('Landmarks_Range_bearing' , 'ObservationModel_class')
    case 'Multi Robot Sensing'
        addpath(genpath('All_observation_models/Multi_robot_sensing')); % In this line we import everything inside the "All_observation_models/multi_robot_sensing" directory.
    case 'Two robots with no communication'
        typeDef('Two_robots_no_communication','ObservationModel_class')
        %addpath(genpath('All_observation_models/Two_robots_no_communication')); % In this line we import everything inside the "All_observation_models/multi_robot_sensing" directory.
    case 'Two robots with communication'
        addpath(genpath('All_observation_models/Two_robots_with_communication')); % In this line we import everything inside the "All_observation_models/multi_robot_sensing" directory.
    case 'n-arm Manipulator'
        typeDef('Planar_manipulator_wall_sensing_deadzone','ObservationModel_class');
    case 'Dynamical n-arm Manipulator'
        typeDef('Dyn_manipulator_wall_sensing_deadzone','ObservationModel_class');
    case '3D Landmark (range and bearing)'
        typeDef('Landmarks_3D_Range_bearing','ObservationModel_class');
    case 'Full state information, additive Gaussian noise'
        typeDef('Full_state_additive_Gaussian','ObservationModel_class');
end

%=========== Planning Problem (Solver) Parameters
switch par_new.planning_problem_param.solver
    case 'Stationary LQG-based FIRM'
        typeDef('Point_PRM_class', 'PRM_class')
        typeDef('Point_stabilizer_SLQG_class', 'stabilizer_class')
    case 'Periodic LQG-based FIRM'
        typeDef('Orbit_PRM_class', 'PRM_class')
        typeDef('Orbit_stabilizer_PLQG_class', 'stabilizer_class')
    case  'DFL-and-SKF-based FIRM'
        typeDef('Point_PRM_class', 'PRM_class')
        typeDef('Point_stabilizer_DFL_SKF', 'stabilizer_class')
    otherwise
        error('not implemented yet')
end






% if strcmpi(par_new.selected_simulator,'Embedded Simulator')
%     typeDef('EmbeddedSimulator' , 'Simulator')
% elseif strcmpi(par_new.selected_simulator,'V-Rep Simulator')
%     typeDef('VRepSimulator' , 'Simulator')
% end
% %=========== Motion Model Parameters
% if strcmpi(par_new.selected_motion_model,'Multi RandomWalk robots')
%     typeDef('multi_robot_positional_state' , 'state')
%     typeDef('multi_robot_positional_belief' , 'belief')
%     typeDef('RandomWalk' , 'MotionModel_class')
% elseif strcmpi(par_new.selected_motion_model,'Omni-directional three wheel robot')
%     typeDef('planar_robot_XYTheta_state' , 'state')
%     typeDef('planar_robot_XYTheta_belief' , 'belief')
%     typeDef('Omni_directional_robot' , 'MotionModel_class')
% elseif strcmpi(par_new.selected_motion_model,'Unicycle')
%     typeDef('planar_robot_XYTheta_state' , 'state')
%     typeDef('planar_robot_XYTheta_belief' , 'belief')
%     typeDef('Unicycle_robot' , 'MotionModel_class')
% elseif strcmpi(par_new.selected_motion_model,'Multi Omni-directional robots')
%     addpath(genpath('All_state_classes/Multi_planar_robots_XYTheta')); % In this line we import everything inside the "All_motion_models/Omni_directional_robot" directory.
%     addpath(genpath('All_belief_classes/Multi_planar_robots_XYTheta')); % In this line we import everything inside the "All_motion_models/Omni_directional_robot" directory.
%     addpath(genpath('All_motion_models/Multi_Omni_directional_robots')); % In this line we import everything inside the "All_motion_models/Omni_directional_robot" directory.
% elseif strcmpi(par_new.selected_motion_model,'Revolute joint 8arm manipulator')
%     typeDef('revolute_joint_manipulator_state' , 'state')
%     typeDef('revolute_joint_manipulator_belief' , 'belief')
%     typeDef('Revolute_joint_manipulator' , 'MotionModel_class')
% elseif strcmpi(par_new.selected_motion_model,'Dynamical planar 8arm manipulator')
%     typeDef('Planar_dyn_manipulator_state' , 'state')
%     typeDef('Planar_dyn_manipulator_belief' , 'belief')
%     typeDef('Dynamical_planar_manipulator' , 'MotionModel_class')
% elseif strcmpi(par_new.selected_motion_model,'FixedWing Aircraft')
%     typeDef('Six_DOF_robot_state' , 'state')
%     typeDef('Six_DOF_robot_belief' , 'belief')
%     typeDef('Aircraft_Kinematic' , 'MotionModel_class')
% elseif strcmpi(par_new.selected_motion_model,'Kuka YouBot Base')
%     typeDef('planar_robot_XYTheta_state' , 'state')
%     typeDef('planar_robot_XYTheta_belief' , 'belief')
%     typeDef('youbot_base' , 'MotionModel_class')
% elseif strcmpi(par_new.selected_motion_model,'Quadrotor')
%     typeDef('Quadrotor_state' , 'state')
%     typeDef('Quadrotor_belief' , 'belief')
%     typeDef('Quadrotor_MM' , 'MotionModel_class')
% end


% %=========== Observation Model Parameters
% if strcmpi(par_new.selected_observation_model,'Three robot good-poor GPS no comm')
%         typeDef('Three_robot_good_poor_GPS_no_comm','ObservationModel_class')
% elseif strcmpi(par_new.selected_observation_model,'Laser Scanner (range bearing)')
%         typeDef('Landmarks_Range_bearing_laser','ObservationModel_class')
%     
% 
% elseif strcmpi(par_new.selected_observation_model,'Three robot good-poor GPS with comm')
%         typeDef('Three_robot_good_poor_GPS_with_comm','ObservationModel_class')
% elseif strcmpi(par_new.selected_observation_model,'Landmark (range and bearing) sensing')
%     typeDef('Landmarks_Range_bearing' , 'ObservationModel_class')
% elseif strcmpi(par_new.selected_observation_model,'Multi Robot Sensing')
%     addpath(genpath('All_observation_models/Multi_robot_sensing')); % In this line we import everything inside the "All_observation_models/multi_robot_sensing" directory.
% elseif strcmpi(par_new.selected_observation_model,'Two robots with no communication')
%     typeDef('Two_robots_no_communication','ObservationModel_class')
%     %addpath(genpath('All_observation_models/Two_robots_no_communication')); % In this line we import everything inside the "All_observation_models/multi_robot_sensing" directory.
% elseif strcmpi(par_new.selected_observation_model,'Two robots with communication')
%     addpath(genpath('All_observation_models/Two_robots_with_communication')); % In this line we import everything inside the "All_observation_models/multi_robot_sensing" directory.
% elseif strcmpi(par_new.selected_observation_model,'n-arm Manipulator')
%     typeDef('Planar_manipulator_wall_sensing_deadzone','ObservationModel_class');
% elseif strcmpi(par_new.selected_observation_model,'Dynamical n-arm Manipulator')
%     typeDef('Dyn_manipulator_wall_sensing_deadzone','ObservationModel_class');
% elseif strcmpi(par_new.selected_observation_model,'3D Landmark (range and bearing)')
%     typeDef('Landmarks_3D_Range_bearing','ObservationModel_class');
% elseif strcmpi(par_new.selected_observation_model,'Full state information, additive Gaussian noise')    
%     typeDef('Full_state_additive_Gaussian','ObservationModel_class');
% end
% 



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
