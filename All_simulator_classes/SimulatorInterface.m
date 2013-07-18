classdef SimulatorInterface
    % This class encapsulates the state of the system.
    properties (Abstract)
        simulatorName; % name of the simulator in use
    end
    
    methods
        % 1) constructor
        function obj = SimulatorInterface()
        end
    end
    methods (Abstract)
        
        % 2) initialize : initializes the simulator 
        obj = initialize(obj)
        % 3)SetRobot : change robot parameters 
        obj = setRobot(obj)
        % 4)GetRobot : get robot parameters
        obj = getRobot(obj)
        % 5) Refresh : 
        obj = refresh(obj)        
        % 6) stopRun (not sure about this one)
        obj = simStop(obj)  
        % 7) evolve : evolve robot
        x_new = evolve(u)
    end
end