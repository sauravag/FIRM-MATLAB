classdef obstacles_class < handle
    %OBSTACLES Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        tmp_prop = obstacles_class.costant_property_constructor();  % I use this technique to initialize the costant properties in run-time. If I can find a better way to do it, I will update it, as it seems a little bit strange.
        map = obstacles_class.tmp_prop.map
        obst = obstacles_class.tmp_prop.obst;
        plot_3D_flag = obstacles_class.tmp_prop.plot_3D_flag;
        obstPlotHandle = obstacles_class.tmp_prop.obstPlotHandle;
        face_color = obstacles_class.tmp_prop.face_color
        face_light = obstacles_class.tmp_prop.face_light
        edge_color = obstacles_class.tmp_prop.edge_color
        edge_width = obstacles_class.tmp_prop.edge_width
        top_height_3D = obstacles_class.tmp_prop.top_height_3D  % hight of the top of 3D obstacles
        bottom_height_3D = obstacles_class.tmp_prop.bottom_height_3D; % hight of the bottom of 3D obstacles        face_light = tmp_prop
        edge_light = obstacles_class.tmp_prop.edge_light;
        obst_2D_color = obstacles_class.tmp_prop.obst_2D_color; % this is only used when 2D obstacles are drawn (for example when user is importing them)
        vObj = obstacles_class.tmp_prop.vObj; % variable containing all the vertices in the obj file of the environment
        fObj = obstacles_class.tmp_prop.fObj; % variable containing all the faces in the obj file of the environment
    end
    properties (Access = private)
        boundary_handle = obstacles_class.tmp_prop.boundary_handle;
        fill_handle = obstacles_class.tmp_prop.fill_handle;
    end
    methods (Static = true)
        function temporary_props = costant_property_constructor()
            obstacleStructure = obstacles_class.get_obstacles;
            temporary_props.obst = obstacleStructure.obst;
            temporary_props.obstPlotHandle = obstacleStructure.obstPlotHandle;
            temporary_props.map = obstacleStructure.map;
            temporary_props.vObj = obstacleStructure.vObj;
            temporary_props.fObj = obstacleStructure.fObj;
            temporary_props.boundary_handle = obstacleStructure.boundary_handle;
            temporary_props.fill_handle = obstacleStructure.fill_handle;
            temporary_props.plot_3D_flag = user_data_class.par.sim.Lighting_and_3D_plots;
            temporary_props.face_color = [.6,.2 0];
            temporary_props.edge_color =  temporary_props.face_color*0.5;
            temporary_props.edge_width = 0.5;
            temporary_props.top_height_3D = user_data_class.par.sim.top_obstacle_height_3D; % hight of the top of 3D obstacles
            temporary_props.bottom_height_3D = user_data_class.par.sim.bottom_obstacle_height_3D; % hight of the bottom of 3D obstacles
            temporary_props.face_light = 'phong';
            temporary_props.edge_light = 'phong';
            temporary_props.obst_2D_color = temporary_props.face_color; % this is only used when 2D obstacles are drawn (for example when user is importing them)
        end
        function tmp_prop = get_obstacles()
            Man_Obst = user_data_class.par.sim.intractive_obst;
            LoadFileName = user_data_class.par.environmentFile;
            if Man_Obst == 0
                if exist(LoadFileName,'file')
                    tmp_prop = obstacles_class.load(LoadFileName);
                    %                     tmp_prop.fill_handle = obstacles_class.draw(tmp_prop.obst); % since we have the class is a handle class, we do NOT need to return the obj.
                    %                     tmp_prop.boundary_handle = []; % in loading obstacles they do not have boundaries.
                    %                     tmp_prop.fill_handle = [];
                else
                    disp(['File : ',LoadFileName,' does not exist. If you want to\n '])
                    disp('switching to manual. ')
                    Man_Obst = 1;
                end
            end
            if Man_Obst == 1
                tmp_prop = obstacles_class.request(LoadFileName);
                Obst_vertices = tmp_prop.obst; %#ok<NASGU>
            end
        end
        function tmp_prop = load(LoadFileName,varargin)
            %                 load(LoadFileName,'Obst_vertices')
            % Reading the obj file containing the map and obstacles
            objOutput=read_wobj(LoadFileName);
            ObstVertices = objOutput.vertices;
            if nargin==2
                figureHandle = varargin{1};
                figure(figureHandle );
            else
                figureHandle = gcf;
                figure(figureHandle);
            end
            axis(user_data_class.par.sim.env_limits)
            title({'Please mark the vertices of polygonal obstacles'},'fontsize',14)
            ib=0;
            tmp_prop.boundary_handle = [];
            tmp_prop.fill_handle = [];
            
            
            tmp_prop.vObj = ObstVertices; % variable containing all the vertices in the obj file;
            fObj = []; % variable containing all faces of the objects in the obj file
            cprintf('Blue','Reading objects from %s \n',LoadFileName)
            tmp_prop.map = [];
            for idx_obj =1:numel(objOutput.objects)
                if (strcmp(objOutput.objects(idx_obj).type,'g')) % this shows that the structure contains the name of an object
                    if user_data_class.par.sim.verboseFlag
                        cprintf('Red','object name : %s \n',objOutput.objects(idx_obj).data)
                    end
                elseif strcmp(objOutput.objects(idx_obj).type,'f') % this is an actual face structure
%                     currentObjectVertIndices =unique(objOutput.objects(idx_obj).data.vertices(:));
                    currentObjectFaces =(objOutput.objects(idx_obj).data.vertices);
                    
                    fObj= [fObj;currentObjectFaces];
                    laserPlane = createPlane([0 0 0.0975], [0 0 1]);
                    % the intersection of the plane with the mesh sometimes
                    % gives more than 2 points on a line segemtn
                    polys = intersectPlaneMesh(laserPlane, ObstVertices, currentObjectFaces);
                    
                    % those points are discarded here if a point is not a corner then it is discarded  
                    B = isParallel3d( circshift(polys{1},[1 0]) - polys{1}, ...
                         polys{1} - circshift(polys{1},[-1 0]),0.001 ); % this line discards non corner points
                    
                    corners = polys{1}(~B,:);
                    tmp_prop.map = [tmp_prop.map,corners(:,1:2)'];
                    
                    
%                     vertPosX = [ObstVertices(currentObjectVertIndices,1);ObstVertices(currentObjectVertIndices(1),1)];
%                     vertPosY = [ObstVertices(currentObjectVertIndices,2);ObstVertices(currentObjectVertIndices(1),2)];
                    axis(user_data_class.par.sim.env_limits)
                    h_obs = patch_display(struct('vertices',ObstVertices,'faces',currentObjectFaces));
%                     h_obs=impoly(gca,polys{1}(:,1:2));
                    
                    %                     h_obs=impoly(gca,[vertPosX,vertPosY]);
%                     tmp_prop.boundary_handle = [tmp_prop.boundary_handle, h_obs];
                    if isempty(h_obs) && ib==0
                        tmp_prop.obst=[];
                        break
                        % % %                     elseif isempty(h_obs) && ib~=0
                        % % %                         tmp_prop.obst = inputed_obstacles;
                        % % %                         break
                    else
                        ib=ib+1;
%                         inputed_obstacles{ib} = h_obs.getPosition; %#ok<AGROW>
                          inputed_obstacles(ib).corners = corners ; %#ok<AGROW>
                          inputed_obstacles(ib).objectFaces = currentObjectFaces ; %#ok<AGROW>
%                         %         [inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2)] = poly2cw(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2));  % ordering the polygon vertices in a clockwise order, if they are not already. % This is gonna be important in drawing 3D version of obstacles, or in projecting light on scene.
%                         fill_color_handle = fill(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2),'r');
%                         tmp_prop.fill_handle = [tmp_prop.fill_handle, fill_color_handle];
                    end
                    %                     bb= obstacles_class.compute_boundary(objOutput.objects(idx_obj).data.vertices )
                end
            end
            tmp_prop.fObj = fObj;
            tmp_prop.obstPlotHandle = figureHandle;
            tmp_prop.obst = inputed_obstacles;
            tmp_prop.boundary_handle = [];
            
        end
        function tmp_prop = request(name)
            %             Obstacles = obstacles_class; %#ok<NASGU> % The object "Obstacles" is never used. This line only cause the "Constant" properties of the "obstacles_class" class to be initialized.
            figure(gcf);
            axis(user_data_class.par.sim.env_limits)
            title({'Please mark the vertices of polygonal obstacles'},'fontsize',14)
            ib=0;
            tmp_prop.boundary_handle = [];
            tmp_prop.fill_handle = [];
            while true
                axis(user_data_class.par.sim.env_limits)
                h_obs=impoly;
                tmp_prop.boundary_handle = [tmp_prop.boundary_handle, h_obs];
                if isempty(h_obs) && ib==0
                    tmp_prop.obst=[];
                    break
                elseif isempty(h_obs) && ib~=0
                    tmp_prop.obst = inputed_obstacles;
                    break
                else
                    ib=ib+1;
                    inputed_obstacles{ib} = h_obs.getPosition; %#ok<AGROW>
                    %         [inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2)] = poly2cw(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2));  % ordering the polygon vertices in a clockwise order, if they are not already. % This is gonna be important in drawing 3D version of obstacles, or in projecting light on scene.
                    fill_color_handle = fill(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2),'b');
                    tmp_prop.fill_handle = [tmp_prop.fill_handle, fill_color_handle];
                end
            end
            %             name = './test_delete_env8.obj';
            fid = fopen(name,'w');
            numOfVertices = 0;
            for idx = 1:numel(inputed_obstacles)
                %     inputed_obstacles{idx}
                
                numCurrentVertices = size(inputed_obstacles{idx},1);
                C = [(1:(numCurrentVertices-1))' (2:numCurrentVertices)'; numCurrentVertices 1];
                
                dt = DelaunayTri(inputed_obstacles{idx}(:,1), inputed_obstacles{idx}(:,2), C);
                io = dt.inOutStatus();
                FV.vertices=[dt.X,zeros(size(dt.X,1),1)];
                FV.faces=dt.Triangulation(io,:);
                FV.faces = FV.faces + numOfVertices;
                vertface2obj(FV.vertices,FV.faces,num2str(idx),fid)
                numOfVertices = numCurrentVertices + numOfVertices;
                
            end
            fclose(fid);
        end
        function plot_handle = draw()
            plot_handle = [];
            for ob_ctr = 1:numel(obstacles_class.obst)
                if 1% obstacles_class.plot_3D_flag== 1
                tmp_handle = patch_display(struct('vertices',obstacles_class.vObj,'faces',obstacles_class.obst(ob_ctr).objectFaces));

                    plot_handle = [plot_handle,tmp_handle]; %#ok<AGROW>
                    
                else
                    fill_color_handle = fill(obstacles_class.obst(ob_ctr).corners(1,:),obstacles_class.obst(ob_ctr).corners(2,:),obstacles_class.obst_2D_color);
                    set(fill_color_handle,'LineWidth',obstacles_class.edge_width,'EdgeColor',obstacles_class.edge_color,'Facecolor',obstacles_class.face_color);
                    %                 set(fill_color_handle,'LineWidth',2,'EdgeColor',0.1*ones(1,3),'EdgeLighting','flat','FaceLighting','phong','Facecolor',0.5*ones(1,3),'BackFaceLighting','reverselit');
                    plot_handle = [plot_handle, fill_color_handle]; %#ok<AGROW>
                end
            end
        end
        function [boundary]= compute_boundary(face)
            % compute_boundary - compute the vertices on the boundary of a 3D mesh
            %
            %   boundary=compute_boundary(face);
            %
            %   Copyright (c) 2007 Gabriel Peyre
            
            
            
            if size(face,1)<size(face,2)
                face=face';
            end
            
            nvert=max(max(face));
            nface=size(face,1);
            
            A=sparse(nvert,nvert);
            for i=1:nface
                
                f=face(i,:);
                A(f(1),f(2))=A(f(1),f(2))+1;
                A(f(1),f(3))=A(f(1),f(3))+1;
                A(f(3),f(2))=A(f(3),f(2))+1;
            end
            A=A+A';
            
            for i=1:nvert
                u=find(A(i,:)==1);
                if ~isempty(u)
                    boundary=[i u(1)];
                    break;
                end
            end
            
            s=boundary(2);
            i=2;
            while(i<=nvert)
                u=find(A(s,:)==1);
                if length(u)~=2
                    warning('problem in boundary');
                end
                if u(1)==boundary(i-1)
                    s=u(2);
                else
                    s=u(1);
                end
                if s~=boundary(1)
                    boundary=[boundary s];
                else
                    break;
                end
                i=i+1;
            end
            
            if i>nvert
                warning('problem in boundary');
            end
        end
    end
end

