classdef PRM_3D_planar < PRM_interface
    %PRM_CLASS encapsulates the Probabilistic RoadMap class
    
    properties (Access = private)
        num_2Dnodes;
        nodes_2D;
        edges_2D_list;
        edges_2D_matrix;
    end
    
    
    properties (Access = private)
        % following properties are defined as the abstract properties in the
        % supperclass. However, since in Matlab private properties are not
        % inherited, we have have to rewrite them consistently in all
        % subclasses.
        edges_plot_handle = [];
    end
    
    methods
        function obj = PRM_class(~)
            % See the superclass; The constructor of the superclass, i.e., "PRM_interface" is
            % automatically called.
        end
        function obj = draw(obj)
            % This function draws the PRM graph, in a 2D space. If no "plot properties"
            % are specified by the user, the default "PRM plot properties"
            % will be used.
            old_prop = obj.set_figure();
            % The following initializations are necessary. Because the initialization in the "property definition" is not enough, when we loading an existing object of this class, that initialization does not happen.
            obj.edges_plot_handle = [];
            % retrieve PRM parameters provided by the user
            edge_spec = obj.par.edge_spec;
            edge_width = obj.par.edge_width;
            node_text_flag = obj.par.PRM_node_text;
            varargin_3D_node_props = obj.par.PRM_node_plot_properties;
            % some variable definitions
            N_nodes=obj.num_nodes;
            Edges = obj.edges_matrix;
            Nodes_val = {obj.nodes.val};
            % draw edges
            for p=1:N_nodes
                for q=[1:p-1,p+1:N_nodes] % note that the edges are not bi-directional. So, there may be an edge from node j to i, but not from i to j.
                    if Edges(p,q)==1
                        tmp_handle = ...
                            plot([Nodes_val{p}(1),Nodes_val{q}(1)],[Nodes_val{p}(2),Nodes_val{q}(2)],edge_spec,'linewidth',edge_width);
                        obj.edges_plot_handle = [obj.edges_plot_handle,tmp_handle];
                        % plot([Nodes_val{p}(1),Nodes_val{q}(1)],[Nodes_val{p}(2),Nodes_val{q}(2)],'-or','linewidth',2,'markersize',3,'markerfacecolor','r');
                    end
                end
            end
            % Draw Nodes
            for p=1:N_nodes
                if  node_text_flag == 1
                    obj.nodes(p) = obj.nodes(p).draw(varargin_3D_node_props{:},'text',num2str(p));
                else
                    obj.nodes(p) = obj.nodes(p).draw(varargin_3D_node_props{:});
                end
            end
            obj.reset_figure(old_prop);
        end
        function obj = delete_plot(obj)
            for p = 1:length(obj.nodes)
                obj.nodes(p) = obj.nodes(p).delete_plot();
            end
            disp('Following line must be uncommented ASAP! there is a problem with this line')
%             delete(obj.edges_plot_handle);
            obj.edges_plot_handle = [];
        end
        function obj = load(obj) %#ok<MANU>
            % This function loads the existing PNPRM from the
            % "LoadFileName". If there is no PNPRM present in the
            % "LoadFileName", an error will occur.
            LoadFileName = user_data_class.par.LoadFileName;
            load(LoadFileName,'PRM')
            obj = PRM;
        end
        function obj = save(obj)
            SaveFileName = user_data_class.par.SaveFileName;
            PRM = obj; %#ok<NASGU>
            save(SaveFileName,'PRM','-append')
        end
        function obj = request_construct_and_draw(obj)
            obj = obj.request_2D_nodes_and_construct_random_3D_nodes();
            obj = obj.construct_edges();
            obj = obj.delete_plot(); % This is to delete the points that user has selected already. We will re-draw them in the next line of code.
            obj = obj.draw();
        end
        function obj = add_a_node_and_its_sequel(obj,new_node)
            % first we increase the number of nodes by one
            obj.num_2Dnodes = obj.num_2Dnodes + 1;
            obj.num_nodes = obj.num_nodes + 1;
            % assign an index to the new 2D node
            new_2Dnode_ind = obj.num_2Dnodes;
            new_3D_node_ind = obj.num_nodes; %#ok<NASGU>
            % add it to the list of 2D nodes
            obj.nodes_2D = [obj.nodes_2D,new_node.val(1:2)];
            obj.nodes(end+1) = new_node;
            % we compute the 2D neighbors of the newly added 2D node.
            neighbors_of_new = obj.find_2D_neighbors(new_2Dnode_ind); % Note that these are 2D neighbors of the new node.
            
            for j2D = neighbors_of_new % "neighbors_of_new" must be a row vector for this loop to work.
                intersection = obj.is_edge_obst_collide(new_2Dnode_ind,j2D);
                if ~intersection
                    obj.edges_2D_list = [obj.edges_2D_list; [new_2Dnode_ind,j2D]];
                    obj.edges_2D_matrix(new_2Dnode_ind,j2D) = 1;
                    obj.edges_list = obj.edges_2D_list;
                    obj.edges_matrix = obj.edges_2D_matrix;
                    % VERY IMPORTANT: Note that the "edges_2D_matrix" and "edges_matrix" are not
                    % symmetric anymore, as we do NOT consider an edge from "j2D"
                    % to "new_node_ind".
                end
            end
            % since "obj.edges_matrix" may not be square after above
            % opertations, we make it square here.
            tmp = zeros(obj.num_nodes,obj.num_nodes);
            tmp(1:size(obj.edges_matrix,1),1:size(obj.edges_matrix,2)) = obj.edges_matrix;
            obj.edges_matrix = tmp;
            % following is an extremely inefficient!!!!! drawing procedure
            obj = obj.delete_plot();
            obj = obj.draw();
        end
        function feedback_plot_handle = draw_feedback_pi(obj, feedback_pi, selected_node_indices)
            % This function has not been updated after the latest changes.
            
            % the selected nodes are a set of nodes that the feedback pi is
            % only drawn for them. This is for uncluttering the figure.
            if ~exist('selected_node_indices', 'var') % if there is no "selected nodes", we draw the "feedback pi" for all nodes.
                selected_node_indices = 1:obj.num_nodes;
            elseif isvector(selected_node_indices) % the "selected_node_indices" has to be a row vector. So, here we make the code robust to column vector as well.
                selected_node_indices = reshape(selected_node_indices,1,length(selected_node_indices));
            end
            feedback_plot_handle = [];
            num_2Dedges = length(obj.edges_2D_list);
            feedback_text_handle = zeros(1,num_2Dedges);
            for i = selected_node_indices
                start = obj.nodes(i).val(1:2); % from now on, in this function, we only consider 2D position of the nodes.
                j = feedback_pi(i); % j is the next node for node i, based on "feedback pi".
                if isnan(j) % the feedback_pi on the goal node return "nan"
                    continue
                end
                [~,edge_num_3D] = intersect(obj.edges_list,[i,j],'rows'); % this function returns the number of 3D edge, whose start and end nodes are i and j, respectively.
                edge_num =obj.corresponding_2D_edges(edge_num_3D); % this line returns the number of corresponding 2D edge.
                % in the following we draw a paraller line to the edge,
                % through which we want to illustrate the feedback "pi".
                final = obj.nodes(j).val(1:2);
                parallel_vector_normalized = (final - start)/norm(final - start);
                perpendicular_vector_normalized = [ parallel_vector_normalized(2); - parallel_vector_normalized(1)];
                shiftet_dist = 1;
                length_of_parallel =  norm(final - start)/3;    % length of the dotted parallel line
                offset_from_start = (norm(final - start) - length_of_parallel)/2;
                start_new = start + shiftet_dist * perpendicular_vector_normalized + offset_from_start  * parallel_vector_normalized;
                final_new = final + shiftet_dist * perpendicular_vector_normalized - offset_from_start  * parallel_vector_normalized;
                plot([start_new(1), final_new(1)],[start_new(2), final_new(2)], '--r');
                
                % in the following we plot the small triangle (called arrow
                % here) on the dotted parallel line
                middle = (start_new + final_new)/2;
                arrow_size = length_of_parallel/5;
                arrow_head = middle + parallel_vector_normalized*arrow_size/2;
                bottom_mid = middle - parallel_vector_normalized*arrow_size/2;
                bottom_vertices_outer = bottom_mid + perpendicular_vector_normalized*arrow_size/4;
                bottom_vertices_inner = bottom_mid - perpendicular_vector_normalized*arrow_size/4;
                verts = [arrow_head';bottom_vertices_outer';bottom_vertices_inner'];
                faces = [1  2 3]; % this tells the number of vertices in the patch object. In our case, we have only three vertices and all of them are among the patch vertices.
                patch_handle = patch('Faces',faces,'Vertices',verts,'FaceColor','g','EdgeColor','r');
                feedback_plot_handle = [feedback_plot_handle, patch_handle]; %#ok<AGROW>
                
                % in the following, we write the number of the node on the
                % corresponding parallel lines.
                text_dist = shiftet_dist/2;
                text_position = bottom_vertices_outer + text_dist*perpendicular_vector_normalized;
                text_position(1) = text_position(1) - 0.45; % for some reason MATLAB shifts the starting point of the text a little bit to the right. So, here we return it back.
                if feedback_text_handle (edge_num) ~= 0 % which means some text has already been written for this edge
                    current_text = get(feedback_text_handle (edge_num), 'String');
                    set(feedback_text_handle (edge_num), 'String', [current_text, ', ', num2str(i)])
                else
                    feedback_text_handle (edge_num) = text(text_position(1), text_position(2), num2str(i), 'fontsize',10,'color','r','EdgeColor','g');
                    feedback_plot_handle = [feedback_plot_handle, feedback_text_handle (edge_num)]; %#ok<AGROW>
                end
            end
        end
        function nearest_node_ind = compute_nearest_node_ind(obj,current_node_ind)
            % This function computes the nearest node in the 2D distance
            % sense.
            neighbors = find(obj.edges_matrix(current_node_ind,:));
            dist = nan(1,length(neighbors));
            for i = 1 : length(neighbors)
                dist(i) = norm( obj.nodes(current_node_ind).val(1:2) - obj.nodes(neighbors(i)).val(1:2) );
            end
            [~,min_ind] = min(dist);
            nearest_node_ind = neighbors(min_ind);
        end
    end
    
    methods (Access = private)
        function obj = request_2D_nodes_and_construct_random_3D_nodes(obj)
            i=0;
            obj.num_2Dnodes = 0; % initialization
            old_prop = obj.set_figure();
            title({'Please mark waypoints'},'fontsize',14)
            node_prop_varargin = obj.par.PRM_node_plot_properties;
            obj.nodes = state.empty; % class type initialization
            tmp_circle_handle = [];
            while true
                i=i+1;
                [x_temp,y_temp]=ginput(1);
                if isempty(x_temp)
                    delete(tmp_circle_handle)
                    break
                else
                    x_c(i)=x_temp; %#ok<AGROW>
                    y_c(i)=y_temp; %#ok<AGROW>
                    random_theta(i) = rand*2*pi - pi; %#ok<AGROW> % generates a random orientation between -pi and pi
                    obj.nodes_2D(:,i) = [x_c(i);y_c(i)];
                    obj.num_2Dnodes = obj.num_2Dnodes+1;
                    obj.nodes(i) = state([x_c(i) ; y_c(i) ; random_theta(i)]);
                    obj.nodes(i) = obj.nodes(i).draw(node_prop_varargin{:});
                    % depict the distance from the node that can connect to
                    % other nodes
                    delete(tmp_circle_handle)
                    tmp_th = 0:0.1:2*pi;
                    tmp_circle_handle = plot(obj.par.node2D_neighboring_distance_threshold*cos(tmp_th)+x_c(i),obj.par.node2D_neighboring_distance_threshold*sin(tmp_th)+y_c(i));
                    % in the following we plot the temporary version of the
                    % edges, so that the user can easily choose the PRM
                    % nodes
                    neighbors_of_i2D = obj.find_2D_neighbors(i);
                    for j2D = neighbors_of_i2D % "neighbors_of_i2D" must be a row vector for this loop to work.
                        intersection = obj.is_edge_obst_collide(i,j2D);
                        if intersection == 0
                            plot([x_c(i),x_c(j2D)],[y_c(i),y_c(j2D)]);
                        end
                    end
                end
            end
            obj.num_nodes = length(obj.nodes);
            obj.reset_figure(old_prop);
            title([]);
        end
        function obj = construct_edges(obj)
            obj = obj.construct_2D_edges_distance_based();
            obj = obj.construct_edges();
        end
        function obj = construct_2D_edges_distance_based(obj)
            N_2Dnodes = obj.num_2Dnodes;
            % constructing the "edges_2D_list"
            obj.edges_2D_list = [];
            for i2D=1:N_2Dnodes
                neighbors_of_i2D = obj.find_2D_neighbors(i2D);
                for j2D = neighbors_of_i2D % "neighbors_of_i2D" must be a row vector for this loop to work.
                    intersection = obj.is_edge_obst_collide(i2D,j2D);
                    if intersection == 0
                        obj.edges_2D_list = [obj.edges_2D_list;[i2D,j2D]];
                    end
                end
            end
            % constructing the "edges_2D_matrix"
            num_2D_edges = size(obj.edges_2D_list,1);
            obj.edges_2D_matrix = zeros(N_2Dnodes,N_2Dnodes);
            for i = 1 : num_2D_edges
                obj.edges_2D_matrix(obj.edges_2D_list(i,1),obj.edges_2D_list(i,2)) = 1;
            end
        end
        function obj = construct_edges(obj)
            obj.edges_matrix = obj.edges_2D_matrix;
            obj.edges_list = obj.edges_2D_list;
        end
        function neighbors = find_2D_neighbors(obj,i2D)
            neighbors = [];
            for j2D = [1:i2D-1,i2D+1:obj.num_2Dnodes]
                if norm(obj.nodes_2D(:,i2D)-obj.nodes_2D(:,j2D)) < ...
                        obj.par.node2D_neighboring_distance_threshold
                    neighbors = [neighbors,j2D]; %#ok<AGROW>
                end
            end
        end
        function YesNo = is_edge_obst_collide(obj,i2D,j2D)
            % "is_edge_obst_collide" function, checks if a PRM edge intersects with an obstacle or not.
            Obst=obstacles_class.obst;
            edge_start = obj.nodes_2D(:,i2D);
            edge_end = obj.nodes_2D(:,j2D);
            
            N_obst=size(Obst,2);
            intersection=0;
            for ib=1:N_obst
                X_obs=[Obst{ib}(:,1);Obst{ib}(1,1)];
                Y_obs=[Obst{ib}(:,2);Obst{ib}(1,2)];
                X_edge=[edge_start(1);edge_end(1)];
                Y_edge=[edge_start(2);edge_end(2)];
                [x_inters,~] = polyxpoly(X_obs,Y_obs,X_edge,Y_edge);
                if ~isempty(x_inters)
                    intersection=intersection+1;
                end
            end
            if intersection>0
                YesNo=1;
            else
                YesNo=0;
            end
        end
    end
end