classdef Simulator < SimulatorInterface
    properties
        sceneHierarchy %% an structure containing Scene Hierarchy (objects in the scene like floor , walls) and their children and properties
        robot
        par
        obstacle
        simulatorName = 'Embedded';
        belief
        videoObj
    end
    
    methods
        % constructor
        function obj = Simulator()
            % in constructor we retrive the paraemters of the planning
            % problem entered by the user.
            obj.par = user_data_class.par.sim;
            %             obj.robot = Robot([0;0;0]);
        end
        % initialize : initializes the simulator
        function obj = initialize(obj)
            old_prop = obj.set_figure(); %#ok<NASGU>
            % Following "if" statements cause the code to first construct the existing
            % parts of the environment, and then construct the parts that
            % the user is going to build.
            if user_data_class.par.observation_model_parameters.interactive_OM == 0
                OM = ObservationModel_class; % The object OM is only created for "Constant" properties of the "ObservationModel_class" class to be initialized.
                OM = OM.draw(); %#ok<NASGU>
            end
            if obj.par.intractive_obst == 0
                obj.obstacle = obstacles_class; % The object "Obstacles" is never used. This line only cause the "Constant" properties of the "obstacles_class" class to be initialized.
                obj.obstacle = obj.obstacle.draw();
            end
            
            if obj.par.intractive_obst == 1
                obj.obstacle = obstacles_class; % The object "Obstacles" is never used. This line only cause the "Constant" properties of the "obstacles_class" class to be initialized.
            end
            if user_data_class.par.observation_model_parameters.interactive_OM == 1
                OM = ObservationModel_class; % The object OM is only created for "Constant" properties of the "ObservationModel_class" class to be initialized.
                OM.plot_handle = OM.draw();
            end
            
            % video making
            if obj.par.video == 1;
                [file,path] = uiputfile('OnlinePhaseVideo.avi','Save the runtime video as');
                vidObj = VideoWriter(fullfile(path,file));
                vidObj.Quality = obj.par.video_quality;
                vidObj.FrameRate = obj.par.FrameRate;
                open(vidObj);
                obj.videoObj = vidObj;
            end
            %obj = Environment_construction(obj); % Construct the environment (obstacles, landmarks, PRM)
            if ~strcmpi(obj.par.env_background_image_address,'none') % check to see if the environment has any background picuture or not
                background = imread(obj.par.env_background_image_address);
                smaller_background=imresize(background,obj.par.imageResizeRatio);
                smaller_background = flipdim(smaller_background,1);
                warp(smaller_background); axis on; set(gca,'Ydir','normal');view(0,90)
            end
            if obj.par.Lighting_and_3D_plots == 1
                view(obj.par.viewAngle);
                camlight('right')
                camzoom(obj.par.initialZoomRatio)
            end
            obj.robot = state();
            obj.belief = belief();
        end
        % SetRobot : change robot parameters
        function obj = setRobot(obj,robot)
            
            if ~isfield(obj.robot,'plot_handle') || isempty(obj.robot.plot_handle) % if this is empty, it shows that the robot field is not initialized yet or we have deleted
                % its handle that is we want to dreaw ir wirh a new handle
                if ~isa(robot, 'state'), robot = state(robot); end
                
                obj.robot = robot;
            else
                % otherwose just update the value
                if ~isa(robot, 'state'), newVal = state(robot); end
                
                obj.robot.val = newVal.val;
            end
        end
        % GetRobot : get robot parameters
        function robot = getRobot(obj)
            robot = obj.robot;
        end
        % Refresh :
        function obj = refresh(obj)
            obj.robot = obj.robot.delete_plot();
            obj.robot = obj.robot.draw();
            %             obj.belief = obj.belief.delete_plot();
            %             obj.belief = obj.belief.draw();
        end
        function obj = recordVideo(obj)
            if user_data_class.par.sim.video == 1
                currFrame = getframe(gcf);
                writeVideo(obj.videoObj ,currFrame);
            end
        end
        function b = getBelief(obj)
            b=obj.belief;
        end
        function obj = setBelief(obj,b)
            
            if ~isfield(obj.belief,'ellipse_handle') || ~isempty(obj.belief.ellipse_handle) || ~isempty(obj.belief.est_mean.plot_handle)
                % if any of the belief object's graphics handles are
                % non-empty, we just
                obj.belief.est_cov = b.est_cov;
                obj.belief.est_mean.val = b.est_mean.val;
            else
                obj.belief = b;
            end
        end
        % stopRun (not sure about this one)
        function obj = simStop(obj)
            disp('Simulation Stopped')
        end
        % evolve : evolve robot
        function obj = evolve(obj,u,varargin)
            if nargin==3
                noiseMode = varargin{1};
            else
                noiseMode = 1; % by default we add noise
            end
            if noiseMode
                w = MotionModel_class.generate_process_noise(obj.robot.val,u);
            else
                w = MotionModel_class.zeroNoise;
            end
            obj.robot.val = MotionModel_class.f_discrete(obj.robot.val,u,w);
        end
        
        function z = getObservation(obj, noiseMode)
            % generating observation noise
            if noiseMode
                v = ObservationModel_class.generate_observation_noise(obj.robot.val);
            else
                v = ObservationModel_class.zeroNoise;
            end
            % constructing ground truth observation
            z = ObservationModel_class.h_func(obj.robot.val,v);
        end
        function isCollided = checkCollision(obj)
            isCollided = 0;
        end
        
    end
    
    methods (Access = private)
        function old_prop = set_figure(obj) % This function sets the figure (size and other properties) to values that are needed for landmark selection or drawing.
            figure(gcf);
            if ~strcmpi(obj.par.env_background_image_address,'none') % check to see if the environment has any background picuture or not
                background = imread(obj.par.env_background_image_address);
                smaller_background=imresize(background,obj.par.imageResizeRatio);
                smaller_background = flipdim(smaller_background,1);
                imshow(smaller_background); axis on; set(gca,'Ydir','normal');view(0,90)
            end
            old_prop{1}=get(gca,'NextPlot');hold on; % save the old "NextPlot" property and set it to "hold on" % Note that this procedure cannot be moved into the "set_figure" function.
            old_prop{2}=get(gca,'XGrid'); % save the old "XGrid" property.
            old_prop{3}=get(gca,'YGrid'); % save the old "YGrid" property.
            grid on; % set the XGrid and YGrid to "on".
            if ~isempty(obj.par.figure_position)
                set(gcf,'Position',obj.par.figure_position)
            end
            axis(obj.par.env_limits);
            set(gca,'DataAspectRatio',[1 1 1]); % makes the scaling of different axes the same. So, a circle is shown as a circle not ellipse.
            %             gray_box_position = get(gca, 'OuterPosition');
            %             set(gca, 'Position', gray_box_position);
        end
    end
    
end
