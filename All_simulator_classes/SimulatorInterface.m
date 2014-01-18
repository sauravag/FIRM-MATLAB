classdef SimulatorInterface < handle
    % This class encapsulates the state of the system.
    properties (Abstract)
        simulatorName; % name of the simulator in use
    end
    
    methods
        % constructor
        function obj = SimulatorInterface()
        end
        
        % destructor
        function obj = simDelete(obj)
        end
    end
    methods (Abstract)
        
        % initialize : initializes the simulator 
        obj = initialize(obj)
        % SetRobot : change robot parameters 
        obj = setRobot(obj)
        % GetRobot : get robot parameters
        obj = getRobot(obj)
        % Refresh : 
        obj = refresh(obj)        
        % stopRun (not sure about this one)
        obj = simStop(obj)  
        % evolve : evolve robot
        x_new = evolve(u)
        % reocrd video
        obj = recordVideo(obj)  
    end
end