classdef Simulator < SimulatorInterface
    properties
        sceneHierarchy %% an structure containing Scene Hierarchy (objects in the scene like floor , walls) and their children and properties
        robot
        par;
        obstacle
        simulatorName = 'Embedded';
    end

    methods
        % 1) constructor
        function obj = Simulator()
            % in constructor we retrive the paraemters of the planning
            % problem entered by the user.
            obj.par = user_data_class.par.sim;
%             obj.robot = Robot([0;0;0]);
            obj.obstacle = obstacles_class;
        end
        % 2) initialize : initializes the simulator
        function obj = initialize(obj)
            
            % read object from .obj file
%             obj.obstacle.get
            % video making
            if obj.par.video == 1;
                global vidObj; %#ok<TLEV>
                vidObj = VideoWriter([obj.par.video_directory,'\OnlinePhase_video.avi']);
                vidObj.Quality = obj.par.video_quality;
                vidObj.FrameRate = obj.par.FrameRate;
                open(vidObj);
            end
%             obj = Environment_construction(obj); % Construct the environment (obstacles, landmarks, PRM)
            if ~strcmpi(obj.par.env_background_image_address,'none') % check to see if the environment has any background picuture or not
                background = imread(obj.par.env_background_image_address);
                smaller_background=imresize(background,Simulator.par.imageResizeRatio);
                smaller_background = flipdim(smaller_background,1);
                warp(smaller_background); axis on; set(gca,'Ydir','normal');view(0,90)
            end
            if obj.par.Lighting_and_3D_plots == 1
                view(Simulator.par.viewAngle);
                camlight('right')
                camzoom(Simulator.par.initialZoomRatio)
            end
            
        end
        % 3)SetRobot : change robot parameters
        function obj = setRobot(obj,robot)
            obj.robot = robot;
        end
        % 4)GetRobot : get robot parameters
        function robot = getRobot(obj)
            robot = obj.robot;
        end
        % 5) Refresh :
        function obj = refresh(obj)
            obj.robot = obj.robot.delete_plot('head','triangle','text');
            obj.robot = obj.robot.draw();
                        

%             disp('Refresh')
        end
        % 6) stopRun (not sure about this one)
        function obj = simStop(obj)
            disp('Simulation Stopped')
        end
        % 7) evolve : evolve robot
        function obj = evolve(obj,u)
            w =  zeros(MotionModel_class.wDim,1);
            obj.robot.val = MotionModel_class.f_discrete(obj.robot.val,u,w);
        end
       
        
    end
    
    methods (Access = private)
        function old_prop = set_figure(obj) % This function sets the figure (size and other properties) to values that are needed for landmark selection or drawing.
            figure(gcf);
            if ~strcmpi(obj.par.env_background_image_address,'none') % check to see if the environment has any background picuture or not
                background = imread(obj.par.env_background_image_address);
                smaller_background=imresize(background,Simulator.par.imageResizeRatio);
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
    
% % % % % % % % end
% % % % % % % % function obj = Environment_construction(obj)
% % % % % % % % old_prop = set_figure(obj); %#ok<NASGU>
% % % % % % % % % Following "if" statements cause the code to first construct the existing
% % % % % % % % % parts of the environment, and then construct the parts that
% % % % % % % % % the user is going to build.
% % % % % % % % if user_data_class.par.observation_model_parameters.interactive_OM == 0
% % % % % % % %     %     OM = ObservationModel_class; % The object OM is only created for "Constant" properties of the "ObservationModel_class" class to be initialized.
% % % % % % % %     %     OM = OM.draw();
% % % % % % % % end
% % % % % % % % if obj.par.intractive_obst == 0
% % % % % % % %     Obstacles = obstacles_class; %#ok<NASGU> % The object "Obstacles" is never used. This line only cause the "Constant" properties of the "obstacles_class" class to be initialized.
% % % % % % % % end
% % % % % % % % if obj.par.interactive_PRM == 0
% % % % % % % %     %         if strcmpi(obj.par.solver,'Periodic LQG-based FIRM')
% % % % % % % %     %             obj.PRM = PNPRM_class;
% % % % % % % %     %             obj.PRM = obj.PRM.draw();
% % % % % % % %     %         else
% % % % % % % %     %     obj.PRM = PRM_class;
% % % % % % % %     %     obj.PRM = obj.PRM.draw();
% % % % % % % %     %         end
% % % % % % % % end
% % % % % % % % 
% % % % % % % % if obj.par.intractive_obst == 1
% % % % % % % %     Obstacles = obstacles_class; %#ok<NASGU> % The object "Obstacles" is never used. This line only cause the "Constant" properties of the "obstacles_class" class to be initialized.
% % % % % % % % end
% % % % % % % % if user_data_class.par.observation_model_parameters.interactive_OM == 1
% % % % % % % %     %     OM = ObservationModel_class; % The object OM is only created for "Constant" properties of the "ObservationModel_class" class to be initialized.
% % % % % % % %     %     OM.plot_handle = ObservationModel_class.tmp_prop.tmp_plot_handle; % the "plot_handle" property is not a constant property. Thus, it has to be assigned to its value here.
% % % % % % % % end
% % % % % % % % if obj.par.interactive_PRM == 1
% % % % % % % %     %         if strcmpi(obj.par.solver,'Periodic LQG-based FIRM')
% % % % % % % %     %             obj.PRM = PNPRM_class;
% % % % % % % %     %         else
% % % % % % % %     %     obj.PRM = PRM_class;
% % % % % % % %     %         end
% % % % % % % % end
end
