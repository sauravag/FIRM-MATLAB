classdef obstacles_class < handle
    %OBSTACLES Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        tmp_prop = obstacles_class.get_obstacles();  % This property is not needed!! I only added it because I could not find any other way to initialize the constant properties. 
        obst = obstacles_class.tmp_prop.obst;
        plot_3D_flag= user_data_class.par.sim.Lighting_and_3D_plots;
        face_color = [.6,.2 0];
        edge_color = obstacles_class.face_color*0.5;
        edge_width = 0.5;
        top_height_3D = 3; % hight of the top of 3D obstacles
        bottom_height_3D = -obstacles_class.top_height_3D; % hight of the bottom of 3D obstacles 
        face_light = 'phong';
        edge_light = 'phong';
        
        obst_2D_color = obstacles_class.face_color; % this is only used when 2D obstacles are drawn (for example when user is importing them)
    end
    properties (Access = private)
        boundary_handle = obstacles_class.tmp_prop.boundary_handle;
        fill_handle = obstacles_class.tmp_prop.fill_handle;
    end
    
    methods (Static = true)
        function tmp_prop = get_obstacles()
            SaveFileName = user_data_class.par.SaveFileName;
            Man_Obst = user_data_class.par.sim.intractive_obst;
            if Man_Obst == 0
                LoadFileName = user_data_class.par.LoadFileName;
                tmp_prop = obstacles_class.load(LoadFileName);
                tmp_prop.fill_handle = obstacles_class.draw(tmp_prop.obst); % since we have the class is a handle class, we do NOT need to return the obj.
                tmp_prop.boundary_handle = []; % in loading obstacles they do not have boundaries.
            else
                tmp_prop = obstacles_class.request();
            end
            Obst_vertices = tmp_prop.obst; %#ok<NASGU>
            save(SaveFileName,'Obst_vertices','-append')
        end
        function tmp_prop = load(LoadFileName)
            load(LoadFileName,'Obst_vertices')
                        %=============================================
%                         disp('Here, we add some specifice obstacles for an specific application (8arm manipulator). This line has to be removed ASAP.')
%                         Obst_vertices{1} = [1.7,0.8 ; 3,0.8 ; 3,1.2 ; 1.7,1.2 ];
%                         Obst_vertices{2} = [-0.65,0.8 ; 0.65,0.8 ; 0.65,1.2 ; -0.65,1.2 ];
%                         Obst_vertices{3} = [-1.7,0.8 ; -3,0.8 ; -3,1.2 ; -1.7,1.2 ];
            %=============================================
%                         disp('Here, we add some specifice obstacles for an specific application (multi-robot). This line has to be removed ASAP.')
%                         Obst_vertices{1} = [-3,5 ; -3,30 ; -30,30 ; -30,5 ];
%                         Obst_vertices{2} = [3,5 ; 3,30 ; 30,30 ; 30,5 ];
%                         Obst_vertices{3} = [17,35 ; 17,40 ; -17,40 ; -17,35 ];
%                         Obst_vertices{4} = [22,35 ; 22,40 ; 35,40 ; 35,35 ];
%                         Obst_vertices{5} = [-22,35 ; -22,40 ; -35,40 ; -35,35 ];
            %=============================================
%             disp('Here, we add some specifice obstacles for an specific application (multi-robot). This line has to be removed ASAP.')
%             Obst_vertices{1} = [-30,-50 ; -30,50 ; -55,50 ; -55,-50 ];
%             Obst_vertices{2} = [30,-50 ; 30,50 ; 55,50 ; 55,-50 ];
            %=============================================
%                         disp('Here, we add some specifice obstacles for an specific application (multi-robot). This line has to be removed ASAP.')
%                         obst_radius = 10;
%                         Base_obst = [-.5,1 ; .5,1 ; 1,.5 ; 1,-.5 ; .5,-1 ; -.5,-1 ; -1,-.5 ; -1,.5]*obst_radius;
%                         ctr = 0;
%                         passage_width = 5;
%                         dist_of_obst = obst_radius*2 + passage_width;
%                         for i_row = -1:1
%                             for j_row = -1:1
%                                 obst_center = [i_row * dist_of_obst ; j_row * dist_of_obst];
%                                 ctr = ctr+1;
%                                 Obst_vertices{ctr} = [Base_obst(:,1)+obst_center(1)  ,  Base_obst(:,2)+obst_center(2)];
%                             end
%                         end
            %=============================================
            tmp_prop.obst = Obst_vertices;
        end
        function tmp_prop = request()
            figure(gcf);
            title({'Please mark the vertices of polygonal obstacles'},'fontsize',14)
            ib=0;
            tmp_prop.boundary_handle = [];
            tmp_prop.fill_handle = [];
            while true
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
                    [inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2)] = poly2cw(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2));  % ordering the polygon vertices in a clockwise order, if they are not already. % This is gonna be important in drawing 3D version of obstacles, or in projecting light on scene.
                    fill_color_handle = fill(inputed_obstacles{ib}(:,1),inputed_obstacles{ib}(:,2),obstacles_class.obst_2D_color);
                    tmp_prop.fill_handle = [tmp_prop.fill_handle, fill_color_handle];
                end
            end
        end
        function plot_handle = draw(obst)
            plot_handle = [];
            top_h = obstacles_class.top_height_3D;
            bottom_h = obstacles_class.bottom_height_3D;
            for ob_ctr = 1:length(obst)
                if 1%obstacles_class.plot_3D_flag== 1
                    % note that the vertices of obstacles are already
                    % ordered in the clockwise direction. So, the following
                    % algorithm to make 3D obstacles, makes sense.
                    x = obst{ob_ctr}(:,1); % x coordinate of the obst. vertices
                    y = obst{ob_ctr}(:,2); % y coordinate of the obst. vertices
                    num_ver = size(obst{ob_ctr},1); % number of vertices of the obstacle "ob_ctr"
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

