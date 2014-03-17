classdef EmbeddedSimulator < SimulatorInterface & handle
    properties
        sceneHierarchy %% an structure containing Scene Hierarchy (objects in the scene like floor , walls) and their children and properties
        robot
        par
        obstacle
        simulatorName = 'Embedded';
        belief
        videoObj
        RayCastObj = [];%opcodemesh(obstacles_class.vObj',obstacles_class.fObj');
        
    end
    
    methods
        % constructor
        function obj = EmbeddedSimulator()
            % in constructor we retrive the paraemters of the planning
            % problem entered by the user.
            obj.par = user_data_class.par.sim;
            embedded_sim_config
            %             obj.robot = Robot([0;0;0]);
            obj.RayCastObj = opcodemesh(obstacles_class.vObj',obstacles_class.fObj');
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
        function obj = getSensorData(obj,sensorName)
            
            
            % Getting laser data
            obj.sensor = obj.sensor.getData();
            
            % Add more cases for other sensors
            
            
        end
        function z = getObservation(obj, noiseMode)
            % generating observation noise
            if noiseMode
                v = ObservationModel_class.generate_observation_noise(obj.robot.val);
            else
                v = ObservationModel_class.zeroNoise;
            end
            % constructing ground truth observation
            intitialTheta = 0*pi/180; % radian. The angle of the first ray of the laser
            endTheta = 180*pi/180; % radian. The angle of the last ray of the laser
            raysPerDegree = 4; % resolution in terms of dnumber of rays per degree
            laserZ = 0.0975; % height of the laser
            resolution = (1/raysPerDegree)*pi/180; % resolution in radians
            robotPose = obj.robot.val; % [x,y,theta]
            from_laser = [robotPose(1),robotPose(2),laserZ];
            theta  = intitialTheta:resolution:endTheta;
            rangeLaser = 4; % meter. the range of the laser scanner
            to_rays = [[rangeLaser.*cos(theta + from_laser(3))]',[rangeLaser.*sin(theta+ from_laser(3))]',repmat(laserZ,length(theta),1) ];
            % from = [-Z(:) Y(:) X(:)]';
            % to = [Z(:) Y(:) X(:)]';
            
            %[hit,d,trix,bary] = opcodemeshmex('intersect',t,from,to-from);
            [hit,d,trix,bary] = obj.RayCastObj.intersect(repmat(from_laser',1,length(theta)),to_rays');
            y= from_laser(2)+rangeLaser.*d'.*sin(theta+ from_laser(3));
            x= from_laser(1)+rangeLaser.*d'.*cos(theta+ from_laser(3));
            z = nan(1,length(theta));
            z(~isnan(d))=laserZ;
            % figure
            % hold on ;
            plot3(x,y,repmat(from_laser(3),1,length(x)),'.r','LineWidth',6)
            scan.x = x.*100;
            scan.y = y.*100;
            new_features_set=hierarchical_feature_extracting(scan,thresholds,'new');
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
