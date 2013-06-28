classdef Robot < state
    properties
        x
        plotHandle
    end
    methods
        function obj = Robot(xInitial)
            obj.x = xInitial;
            obj.plotHandle = [];
        end
        function obj = evolve(obj,u,noiseMode)
            % noiseMode can take two options 'noisy' | 'noiseless'
            if strcmpi(noiseMode,'noisy')
                w = motion_model.generate_process_noise(obj.x,u);
            else
                w = zeros(motion_model.wDim,1);
            end
            obj.x = motion_model.f_discrete(obj.x,u,w);
        end
        function obj = draw(obj)
        end
        function obj = deletePlot(obj)
        end
        function xRobot = getRobot(obj)
            xRobot = obj.x;
        end
        function obj = setRobot(obj,xRobot)
            obj.x = xRobot;
        end
    end
end


