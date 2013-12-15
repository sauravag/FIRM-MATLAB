classdef obstacles_class < handle
    %OBSTACLES Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        tmp_prop = obstacles_class.costant_property_constructor();  % I use this technique to initialize the costant properties in run-time. If I can find a better way to do it, I will update it, as it seems a little bit strange.
        obst = obstacles_class.tmp_prop.obst;
        plot_3D_flag = obstacles_class.tmp_prop.plot_3D_flag;
        face_color = obstacles_class.tmp_prop.face_color
        face_light = obstacles_class.tmp_prop.face_light
        edge_color = obstacles_class.tmp_prop.edge_color
        edge_width = obstacles_class.tmp_prop.edge_width
        top_height_3D = obstacles_class.tmp_prop.top_height_3D  % hight of the top of 3D obstacles
        bottom_height_3D = obstacles_class.tmp_prop.bottom_height_3D; % hight of the bottom of 3D obstacles        face_light = tmp_prop
        edge_light = obstacles_class.tmp_prop.edge_light;
        obst_2D_color = obstacles_class.tmp_prop.obst_2D_color; % this is only used when 2D obstacles are drawn (for example when user is importing them)
    end
    properties (Access = private)
        boundary_handle = obstacles_class.tmp_prop.boundary_handle;
        fill_handle = obstacles_class.tmp_prop.fill_handle;
    end
    methods (Static = true)
        function temporary_props = costant_property_constructor()
            obstacleStructure = obstacles_class.get_obstacles;
            temporary_props.obst = obstacleStructure.obst;
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
        function tmp_prop = load(LoadFileName)
            %                 load(LoadFileName,'Obst_vertices')
            objOutput=read_wobj(LoadFileName);
            ObstVertices = objOutput.vertices;
            tmp_prop.obst = ObstVertices;
            
            figure(gcf);
            axis(user_data_class.par.sim.env_limits)
            title({'Please mark the vertices of polygonal obstacles'},'fontsize',14)
            ib=0;
            tmp_prop.boundary_handle = [];
            tmp_prop.fill_handle = [];
            %             if user_data_class.par_new.sim.verboseFlag
            
            cprintf('Blue','Reading objects from %s \n',LoadFileName)
            %             end
            for idx_obj =1:numel(objOutput.objects)
                if (strcmp(objOutput.objects(idx_obj).type,'g'))
                    if user_data_class.par.sim.verboseFlag
                        cprintf('Red','object name : %d \n',objOutput.objects(idx_obj).data)
                    end
                elseif strcmp(objOutput.objects(idx_obj).type,'f')
                    currentObjectVertIndices =unique(objOutput.objects(idx_obj).data.vertices(:));
                    
                    vertPosX = [ObstVertices(currentObjectVertIndices,1);ObstVertices(currentObjectVertIndices(1),1)];
                    vertPosY = [ObstVertices(currentObjectVertIndices,2);ObstVertices(currentObjectVertIndices(1),2)];
                    axis(user_data_class.par.sim.env_limits)
                    h_obs=impoly(gca,[vertPosX,vertPosY]);
                    tmp_prop.boundary_handle = [tmp_prop.boundary_handle, h_obs];
                    if isempty(h_obs) && ib==0
                        tmp_prop.obst=[];
                        break
                        % % %                     elseif isempty(h_obs) && ib~=0
                        % % %                         tmp_prop.obst = inputed_obstacles;
                        % % %                         break
                    else
                        ib=ib+1;
                        inputed_obstacles{ib} = h_obs.getPosition; %#ok<AGROW>
                        %         [inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2)] = poly2cw(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2));  % ordering the polygon vertices in a clockwise order, if they are not already. % This is gonna be important in drawing 3D version of obstacles, or in projecting light on scene.
                        fill_color_handle = fill(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2),'r');
                        tmp_prop.fill_handle = [tmp_prop.fill_handle, fill_color_handle];
                    end
                end
            end
            tmp_prop.obst = inputed_obstacles;
            
            
            
            
            
            
            %             tmp_prop.boundary_handle = [];
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
            top_h = obstacles_class.top_height_3D;
            bottom_h = obstacles_class.bottom_height_3D;
            for ob_ctr = 1:length(obstacles_class.obst)
                if 1%obstacles_class.plot_3D_flag== 1
                    % note that the vertices of obstacles are already
                    % ordered in the clockwise direction. So, the following
                    % algorithm to make 3D obstacles, makes sense.
                    x = obstacles_class.obst{ob_ctr}(:,1); % x coordinate of the obst. vertices
                    y = obstacles_class.obst{ob_ctr}(:,2); % y coordinate of the obst. vertices
                    num_ver = size(obstacles_class.obst{ob_ctr},1); % number of vertices of the obstacle "ob_ctr"
                    for i = 1:num_ver
                        j = i+1;
                        if j > num_ver, j =1; end % this is to connect the last vertice to the first one
                        side_face_ver_x = [x(i),x(j),x(j),x(i)]; % the x coordinates of the i-th side face
                        side_face_ver_y = [y(i),y(j),y(j),y(i)]; % the x coordinates of the i-th side face
                        side_face_ver_z = [top_h,top_h,bottom_h,bottom_h]; % the x coordinates of the i-th side face
                        tmp_handle = patch(side_face_ver_x,side_face_ver_y,side_face_ver_z,obstacles_class.face_color,'facelight',obstacles_class.face_light,'EdgeColor',obstacles_class.edge_color,'LineWidth',obstacles_class.edge_width,...
                            'EdgeLighting',obstacles_class.edge_light,'FaceLighting',obstacles_class.face_light,'Facecolor',obstacles_class.face_color); % plot the i-th side face of the obstacle cylinder
                        plot_handle = [plot_handle,tmp_handle]; %#ok<AGROW>
                    end
                    tmp_handle = patch(x,y,top_h*ones(num_ver,1),obstacles_class.face_color,'facelight',obstacles_class.face_light ,'EdgeColor',obstacles_class.edge_color,'LineWidth',obstacles_class.edge_width,...
                        'EdgeLighting',obstacles_class.edge_light,'FaceLighting',obstacles_class.face_light,'Facecolor',obstacles_class.face_color); % plot the upper lid of the obstacle cylinder
                    plot_handle = [plot_handle,tmp_handle];     %#ok<AGROW>
                    tmp_handle = patch(x,y,bottom_h*ones(num_ver,1),obstacles_class.face_color,'facelight',obstacles_class.face_light,'EdgeColor',obstacles_class.edge_color,'LineWidth',obstacles_class.edge_width,...
                        'EdgeLighting',obstacles_class.edge_light,'FaceLighting',obstacles_class.face_light,'Facecolor',obstacles_class.face_color); % plot the bottom lid of the obstacle cylinder
                    plot_handle = [plot_handle,tmp_handle]; %#ok<AGROW>
                else
                    fill_color_handle = fill(obst{ob_ctr}(:,1),obst{ob_ctr}(:,2),obstacles_class.obst_2D_color);
                    set(fill_color_handle,'LineWidth',obstacles_class.edge_width,'EdgeColor',obstacles_class.edge_color,'Facecolor',obstacles_class.face_color);
                    %                 set(fill_color_handle,'LineWidth',2,'EdgeColor',0.1*ones(1,3),'EdgeLighting','flat','FaceLighting','phong','Facecolor',0.5*ones(1,3),'BackFaceLighting','reverselit');
                    plot_handle = [plot_handle, fill_color_handle]; %#ok<AGROW>
                end
            end
        end
    end
end

