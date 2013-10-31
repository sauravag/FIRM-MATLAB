classdef PRM_class_nodes_along_edges < PRM_interface
    % This class is a kind of obsolete. We just save it for future
    % references.
    
    properties (Access = private)
        num_2Dnodes;
        nodes_2D;
        edges_2D_list;
        edges_2D_matrix;
        corresponding_2D_node;
        corresponding_2D_edges;
    end
    
    % following properties are defined as the abstract properties in the
    % supperclass. However, since in Matlab private properties are not
    % inherited, we have have to rewrite them consistently in all
    % subclasses.
    properties (Access = private)
         edges_plot_handle = [];
        nodes2D_plot_handle = [];
        nodes3D_plot_handle = [];
        text_handle = [];
    end
    
    methods
        function obj = PRM_class(~)
            % The constructor of the superclass, i.e., "PRM_interface" is
            % automatically called.
        end
        function obj = draw(obj)
            % This function draws the PRM graph, in a 2D space. If no "plot properties"
            % are specified by the user, the default "PRM plot properties"
            % will be used.
            old_prop = obj.set_figure();
            % The following initializations are necessary. Because the initialization in the "property definition" is not enough, when we loading an existing object of this class, that initialization does not happen.
            obj.edges_plot_handle = [];
            obj.nodes2D_plot_handle = [];
            obj.nodes3D_plot_handle = [];
            obj.text_handle = [];
            % retrieve PRM parameters provided by the user
            edge_spec = obj.par.edge_spec;
            edge_width = obj.par.edge_width;
            node_text_flag = obj.par.PRM_node_text;
            varargin_3D_node_props = obj.par.PRM_node_plot_properties;
            % some variable definitions
            N_nodes=obj.num_nodes;
            Edges = obj.edges_matrix;
            Nodes_state = obj.nodes;
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
                    Nodes_state(p) = Nodes_state(p).draw(varargin_3D_node_props{:},'text',num2str(p));
                else
                    Nodes_state(p) = Nodes_state(p).draw(varargin_3D_node_props{:});
                    obj.text_handle = [obj.text_handle,Nodes_state(p).text_handle];
                end
                obj.nodes3D_plot_handle = [obj.nodes3D_plot_handle,Nodes_state(p).tria_handle,Nodes_state(p).head_handle];
            end
            obj.reset_figure(old_prop);
        end
        function obj = delete_plot(obj)
            delete(obj.text_handle);
            obj.text_handle = [];
            delete(obj.nodes3D_plot_handle);
            obj.nodes3D_plot_handle = [];
            delete(obj.edges_plot_handle);
            obj.edges_plot_handle = [];
            delete(obj.nodes2D_plot_handle);
            obj.nodes2D_plot_handle = [];
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
            obj = obj.request_2D_nodes();
            obj = obj.construct();
            obj = obj.draw();
        end
        function obj = add_a_node_and_its_sequel(obj,new_node)
            % first we increase the number of 2D nodes by one
            obj.num_2Dnodes = obj.num_2Dnodes + 1;
            % assign an index to the new 2D node
            new_2Dnode_ind = obj.num_2Dnodes;
            % add it to the list of 2D nodes
            obj.nodes_2D = [obj.nodes_2D,new_node.val(1:2)];
            % for this 2D node, we will have only one corresponding 3D node
            obj.num_nodes = obj.num_nodes + 1;
            new_3D_node_ind = obj.num_nodes;
            obj.nodes(end+1) = new_node;
            % we update the "corresponding_2D_node" property.
            obj.corresponding_2D_node = [obj.corresponding_2D_node,new_2Dnode_ind];
            % we compute the 2D neighbors of the newly added 2D node.
            neighbors_of_new = obj.find_2D_neighbors(new_2Dnode_ind); % Note that these are 2D neighbors of the new node.
            
            
            for j2D = neighbors_of_new % "neighbors_of_new" must be a row vector for this loop to work.
                intersection = obj.is_edge_obst_collide(new_2Dnode_ind,j2D);
                if ~intersection
                    obj.edges_2D_list = [obj.edges_2D_list; [new_2Dnode_ind,j2D]];
                    obj.edges_2D_matrix(new_2Dnode_ind,j2D) = 1;
                    % VERY IMPORTANT: Note that the "edges_2D_matrix" is not
                    % symmetric anymore, as we do NOT consider an edge from "j2D"
                    % to "new_node_ind".
                    new_2D_edge = size(obj.edges_2D_list,1);
                    
                    % in the following line we find a 3D node on the 2D edge.
                    % Note that this 3D node is not added to the list of 3D
                    % nodes. It is only used to find one of the 3D nodes on
                    % the 2D node, that has the least angle different with
                    % the 2D edge.
                    a_3D_node_on_2D_edge = obj.construct_a_single_3D_node(new_2D_edge); 
                    % in the following we compute all the 3D nodes
                    % corresponding to the 2D node j2D.
                    the_3D_nodes_share_j2D = find (obj.corresponding_2D_node == j2D);
                    % in the following, we find which one of the
                    % "the_3D_nodes_share_j2D" is closer to the ideal 3D
                    % node, i.e. "a_3D_node_on_2D_edge", in the "heading
                    % angle" sense.
                    angle_difference = [];
                    for m = 1:length(the_3D_nodes_share_j2D)
                        tmp_node_3D = obj.nodes(the_3D_nodes_share_j2D(m));
                        signed_element_wise_distance = tmp_node_3D.signed_element_wise_dist(a_3D_node_on_2D_edge);  % this basically computes the "signed element-wise distance" between "tmp_node_3D" and "a_3D_node_on_2D_edge"
                        angle_difference(m) = abs(signed_element_wise_distance(end)); % note that the last element (i.e. "end") is the heading angle. % Do not forget the "abs" in computing difference.
                    end
                    [~, min_ind] = min(angle_difference);
                    connecting_3D_node_ind = the_3D_nodes_share_j2D(min_ind);
                    
                    obj.edges_list = [obj.edges_list;[new_3D_node_ind,connecting_3D_node_ind]];
                    obj.edges_matrix(new_3D_node_ind,connecting_3D_node_ind) = 1;
                end
            end
            % since "obj.edges_matrix" may not be square after above
            % opertations, we make it square here.
            tmp = zeros(obj.num_nodes,obj.num_nodes);
            tmp(1:size(obj.edges_matrix,1),1:size(obj.edges_matrix,2)) = obj.edges_matrix;
            obj.edges_matrix = tmp;
            obj = obj.draw();
        end
        function YesNo = has_same_2D_location(obj,i_3D,j_3D)
            % this function returns 1 if the corresponding 2D location of nodes
            % "i_3D" and "j_3D" are same. And it returns 0 otherwise.
            if (obj.corresponding_2D_node(i_3D) == obj.corresponding_2D_node(j_3D))
                YesNo = 1;
            else
                YesNo = 0;
            end
        end
        function feedback_plot_handle = draw_feedback_pi(obj, feedback_pi, selected_node_indices)
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
        function obj = draw_2D_nodes(obj,varargin)
            tmp_handle =  plot(obj.nodes_2D(1,:),obj.nodes_2D(2,:),varargin{:});
            obj.nodes2D_plot_handle = [obj.nodes2D_plot_handle,tmp_handle];
        end
        function obj = request_2D_nodes(obj)
            x_c(1)=0;
            y_c(1)=0;
            i=1;
            tmp_handle = plot(x_c(1),y_c(1));
            obj.nodes2D_plot_handle = [obj.nodes2D_plot_handle,tmp_handle];
            title({'Please mark waypoints'},'fontsize',14)
            axis(user_data_class.par.env_limits); axis square
            while true
                i=i+1;
                [x_temp,y_temp]=ginput(1);
                if isempty(x_temp)
                    break
                else
                    x_c(i)=x_temp; %#ok<AGROW>
                    y_c(i)=y_temp; %#ok<AGROW>
                    tmp_handle = plot(x_c(i-1:i),y_c(i-1:i),'*r','linewidth',3,'markersize',6);
                    obj.nodes2D_plot_handle = [obj.nodes2D_plot_handle,tmp_handle];
                    hold on
                end
            end
            obj.nodes_2D = [x_c;y_c];
            obj.num_2Dnodes = size(obj.nodes_2D,2);
        end
        function obj = construct(obj)
            obj = obj.construct_2D_edges_distance_based();
            obj = obj.construct_nodes();
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
        function obj = construct_2D_edges_KNN_based(obj,num_nearest_neighbors,obstacles_vertices) %#ok<INUSD>
            % This function constructes the 2D PRM edges based on K-nearest
            % neighbor method.
        end
        function obj = construct_nodes(obj)
            obj.num_nodes = size(obj.edges_2D_list,1); % therefore, i-th 3D-node lies on i-th 2D-edge.
            obj.nodes = state.empty; % class type initialization
            for edge_number = 1:obj.num_nodes
                nodeD = obj.construct_a_single_3D_node(edge_number);
                obj.nodes = [obj.nodes;nodeD];
                end_node_j = obj.edges_2D_list(edge_number,2);
                obj.corresponding_2D_node = [obj.corresponding_2D_node,end_node_j];
            end
        end
        function node = construct_a_single_3D_node(obj,edge_number)
            start_node_i = obj.edges_2D_list(edge_number,1);
            end_node_j = obj.edges_2D_list(edge_number,2);
            th = atan2(obj.nodes_2D(2,end_node_j)-obj.nodes_2D(2,start_node_i),obj.nodes_2D(1,end_node_j)-obj.nodes_2D(1,start_node_i));
            node = state([obj.nodes_2D(:,end_node_j);th]);
        end
        function obj = construct_edges(obj)
            N_nodes = obj.num_nodes;
            Nodes = {obj.nodes.val};  % Thus, the Nodes is a cell array
            obj.edges_matrix = zeros(N_nodes,N_nodes);
            disp('AliFW: in the following for loop, all the exact equality checking have to be changed by an "almost equality" checking to avoid numerical round off errors.')
            ctr = 1;
            for p=1:N_nodes
                for q=[1:p-1,p+1:N_nodes]
                    start_2D_ind = obj.corresponding_2D_node(p);
                    end_2D_ind = obj.corresponding_2D_node(q);
                    edge_2D_number = find(start_2D_ind == obj.edges_2D_list(:,1) & end_2D_ind == obj.edges_2D_list(:,2));
                    reverse_edge_2D_number = find(start_2D_ind == obj.edges_2D_list(:,2) & end_2D_ind == obj.edges_2D_list(:,1));
                    % To connect the p-th node3D to q-th node3D: First,
                    % there must be a 2D edge between their corresponding
                    % 2D nodes. Second, the first 3D node, i.e. p, must not
                    % be on the corresponding 2D edge, while the second 3D
                    % node, i.e. q, must be on the corresponding 2D edge.
                    if obj.edges_2D_matrix(start_2D_ind,end_2D_ind) == 1
                        if (p ~= edge_2D_number && p ~= reverse_edge_2D_number) && (q == edge_2D_number || q == reverse_edge_2D_number)
                        obj.edges_matrix(p,q) = 1;
                        obj.edges_list(ctr,:) = [p,q];
                        obj.corresponding_2D_edges(ctr) = edge_2D_number;
                        ctr = ctr + 1;
                        end
                    end
                end
            end
        end
        function obj = load_and_draw_2D_nodes(obj,varargin)
            % This functions has to be among public methods. It is private
            % now, because at this moment we do not call it outside of this
            % class.
            LoadFileName = user_data_class.par.LoadFileName;
            load(LoadFileName,'saved_PRM_2D_nodes')
            obj.nodes_2D = saved_PRM_2D_nodes;
            obj.num_2Dnodes = size(saved_PRM_2D_nodes,2);
            obj = draw_2D_nodes(obj,varargin{:});
        end
        function obj = request_and_save_2D_nodes(obj,varargin)
            % This functions has to be among public methods. It is private
            % now, because at this moment we do not call it outside of this
            % class.
            x_c(1)=0;
            y_c(1)=0;
            i=1;
            tmp_handle = plot(x_c(1),y_c(1),varargin{:});
            obj.nodes2D_plot_handle = [obj.nodes2D_plot_handle,tmp_handle];
            title({'Please mark waypoints'},'fontsize',14)
            while true
                i=i+1;
                [x_temp,y_temp]=ginput(1);
                if isempty(x_temp)
                    break
                else
                    x_c(i)=x_temp; %#ok<AGROW>
                    y_c(i)=y_temp; %#ok<AGROW>
                    tmp_handle = plot(x_c(i-1:i),y_c(i-1:i),'*r','linewidth',3,'markersize',15);
                    obj.nodes2D_plot_handle = [obj.nodes2D_plot_handle,tmp_handle];
                    hold on
                end
            end
            saved_PRM_2D_nodes = [x_c;y_c];
            obj.num_nodes = size(saved_PRM_2D_nodes,2);
            SaveFileName = user_data_class.par.SaveFileName;
            save(SaveFileName,'saved_PRM_2D_nodes','-append')
            obj.nodes_2D = saved_PRM_2D_nodes;
        end
        function neighbors = find_2D_neighbors(obj,i2D)
            neighbors = [];
            for j2D = [1:i2D-1,i2D+1:obj.num_2Dnodes]
                if norm(obj.nodes_2D(:,i2D)-obj.nodes_2D(:,j2D)) < ...
                        user_data_class.par.node2D_neighboring_distance_threshold
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