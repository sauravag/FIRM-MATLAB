classdef Point_PRM_class < PRM_interface
    %Point_PRM_class encapsulates the Probabilistic RoadMap class
  
    
    properties (Access = private)
        % following properties are defined as the abstract properties in the
        % supperclass. However, since in Matlab private properties are not
        % inherited, we have have to rewrite them consistently in all
        % subclasses.
        edges_plot_handle = [];
        max_number_of_nodes = 30;
    end
    
    methods
        function obj = Point_PRM_class(~)
            % See the superclass; The constructor of the superclass, i.e., "PRM_interface" is
            % automatically called.
        end
        function obj = draw(obj)
            % This function draws the PRM graph. If no "plot properties"
            % are specified by the user, the default "PRM plot properties"
            % will be used.
            old_prop = obj.set_figure();
            % The following initializations are necessary. Because the initialization in the "property definition" is not enough, when we loading an existing object of this class, that initialization does not happen.
            obj.edges_plot_handle = [];
            % retrieve PRM parameters provided by the user
            node_text_flag = obj.par.PRM_node_text;
            varargin_node_props = obj.par.PRM_node_plot_properties;
            % some variable definitions
            N_nodes=obj.num_nodes;
            % draw edges
            for i = 1:size(obj.edges_list , 1)
                obj = obj.draw_edge(i);
            end
            % Draw Nodes
            for p=1:N_nodes
                if  node_text_flag == 1
                    obj.nodes(p) = obj.nodes(p).draw(varargin_node_props{:},'text',num2str(p));
                else
                    obj.nodes(p) = obj.nodes(p).draw(varargin_node_props{:});
                end
            end
            obj.reset_figure(old_prop);
        end
        function obj = delete_plot(obj)
            for p = 1:length(obj.nodes)
                obj.nodes(p) = obj.nodes(p).delete_plot();
            end
            delete(obj.edges_plot_handle);
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
        function obj = request_nodes(obj)
            old_prop = obj.set_figure();
            title({'Please select PRM nodes'},'fontsize',14)
            obj.nodes = state.empty; % class type initialization
            tmp_neighb_plot_handle = [];
            for i = 1:obj.max_number_of_nodes
                new_node = state.sample_a_valid_state;
                if isempty(new_node)
                    delete(tmp_neighb_plot_handle)
                    break
                else
                    obj = obj.add_set_of_nodes(new_node);
                    drawnow
                    % depict the distance from the node that can connect to
                    % other nodes
                    delete(tmp_neighb_plot_handle)
                    scale = obj.par.neighboring_distance_threshold;
                    tmp_neighb_plot_handle = new_node.draw_neighborhood(scale);
                end
            end
            obj.reset_figure(old_prop);
            title([]);
        end
        function obj = overwrite_nodes(obj)
        end
        function obj = save(obj)
            SaveFileName = user_data_class.par.SaveFileName;
            PRM = obj; %#ok<NASGU>
            save(SaveFileName,'PRM','-append')
        end
        
        function obj = add_node(obj,new_node)
            % first we increase the number of nodes by one
            obj.num_nodes = obj.num_nodes + 1;
            % assign an index to the new node
            new_node_ind = obj.num_nodes;
            disp(['Adding PRM node ',num2str(new_node_ind), ' to the PRM']);
            % add it to the list of nodes
            obj.nodes(new_node_ind) = new_node;
             % drawing the new node
            node_prop_varargin = obj.par.PRM_node_plot_properties;
            obj.nodes(new_node_ind) = obj.nodes(new_node_ind).draw(node_prop_varargin{:});
            % we compute the neighbors of the newly added node.
            neighbors_of_new = obj.find_neighbors(new_node_ind);
            % we add the edges corresponding to this new node. 
            for j = neighbors_of_new % "neighbors_of_new" must be a row vector for this loop to work.
                % We first consider the edges that goes out of this node
                obj = add_edge(obj, new_node, obj.nodes(j), new_node_ind, j);
                % Then, we consider the edges that comes into this node.
                obj = add_edge(obj, obj.nodes(j), new_node, j, new_node_ind);
            end
            % since "obj.edges_matrix" may not be square after above
            % opertations (if the reverse edges are not collision-free), we make it square here.
            tmp = zeros(obj.num_nodes,obj.num_nodes);
            tmp(1:size(obj.edges_matrix,1),1:size(obj.edges_matrix,2)) = obj.edges_matrix;
            obj.edges_matrix = tmp;
            
            obj.num_stabilizers = obj.num_nodes;
        end
        function obj = add_edge(obj, start_node, end_node, start_node_ind, end_node_ind)
            open_loop_traj = MotionModel_class.generate_VALID_open_loop_point2point_traj(start_node, end_node); % generates open-loop trajectories from "new_node" to j-th PRM node. If the "open_loop_traj" violates constraints (for example collides with obstacles) it returns empty matrix.
            constraint_violation = isempty(open_loop_traj); % this lines check if the "open_loop_traj" violates any constraints or not. For example it checks collision with obstacles.
            if ~constraint_violation
                obj.edges_list = [obj.edges_list; [start_node_ind , end_node_ind] ];
                obj.edges_matrix(start_node_ind , end_node_ind) = 1;
                edge_number = size(obj.edges_list , 1);
                obj.edges(edge_number).x = open_loop_traj.x; obj.edges(edge_number).u = open_loop_traj.u;
                if obj.par.draw_edges_flag, obj = obj.draw_edge(edge_number); end % drawing the newly added node and corresponding edges
                % setting the outgoing edge numbers fromt the "start_node_ind"
                if size(obj.outgoing_edges , 2) < start_node_ind  % This codition basically is this "if ismepty(obj.outgoing_edges{new_node_ind})".
                    obj.outgoing_edges{start_node_ind} = edge_number ;
                else
                    obj.outgoing_edges{start_node_ind} = [obj.outgoing_edges{start_node_ind} , edge_number ];
                end
            end
        end
        
        function feedback_plot_handle = draw_feedback_pi(obj, feedback_pi, selected_node_indices)
            feedback_plot_handle =[];
% % %             error('This function is obsolete. But it can be updated beautifully with the new was of looking to feedback!')
% % %             % This function has not been updated after the latest changes.
% % %             
% % %             % the selected nodes are a set of nodes that the feedback pi is
% % %             % only drawn for them. This is for uncluttering the figure.
% % %             if ~exist('selected_node_indices', 'var') % if there is no "selected nodes", we draw the "feedback pi" for all nodes.
% % %                 selected_node_indices = 1:obj.num_nodes;
% % %             elseif isvector(selected_node_indices) % the "selected_node_indices" has to be a row vector. So, here we make the code robust to column vector as well.
% % %                 selected_node_indices = reshape(selected_node_indices,1,length(selected_node_indices));
% % %             end
% % %             feedback_plot_handle = [];
% % %             num_edges = length(obj.edges_list);
% % %             feedback_text_handle = zeros(1,num_edges);
% % %             for i = selected_node_indices
% % %                 start = obj.nodes(i).val(1:2); % from now on, in this function, we only consider 2D position of the nodes.
% % %                 j = feedback_pi(i); % j is the next node for node i, based on "feedback pi".
% % %                 if isnan(j) % the feedback_pi on the goal node return "nan"
% % %                     continue
% % %                 end
% % %                 [~,edge_num] = intersect(obj.edges_list,[i,j],'rows'); % this function returns the number of PRM edge, whose start and end nodes are i and j, respectively.
% % %                 edge_num =obj.corresponding_2D_edges(edge_num); % this line returns the number of corresponding 2D edge.
% % %                 % in the following we draw a paraller line to the edge,
% % %                 % through which we want to illustrate the feedback "pi".
% % %                 final = obj.nodes(j).val(1:2);
% % %                 parallel_vector_normalized = (final - start)/norm(final - start);
% % %                 perpendicular_vector_normalized = [ parallel_vector_normalized(2); - parallel_vector_normalized(1)];
% % %                 shiftet_dist = 1;
% % %                 length_of_parallel =  norm(final - start)/3;    % length of the dotted parallel line
% % %                 offset_from_start = (norm(final - start) - length_of_parallel)/2;
% % %                 start_new = start + shiftet_dist * perpendicular_vector_normalized + offset_from_start  * parallel_vector_normalized;
% % %                 final_new = final + shiftet_dist * perpendicular_vector_normalized - offset_from_start  * parallel_vector_normalized;
% % %                 plot([start_new(1), final_new(1)],[start_new(2), final_new(2)], '--r');
% % %                 
% % %                 % in the following we plot the small triangle (called arrow
% % %                 % here) on the dotted parallel line
% % %                 middle = (start_new + final_new)/2;
% % %                 arrow_size = length_of_parallel/5;
% % %                 arrow_head = middle + parallel_vector_normalized*arrow_size/2;
% % %                 bottom_mid = middle - parallel_vector_normalized*arrow_size/2;
% % %                 bottom_vertices_outer = bottom_mid + perpendicular_vector_normalized*arrow_size/4;
% % %                 bottom_vertices_inner = bottom_mid - perpendicular_vector_normalized*arrow_size/4;
% % %                 verts = [arrow_head';bottom_vertices_outer';bottom_vertices_inner'];
% % %                 faces = [1  2 3]; % this tells the number of vertices in the patch object. In our case, we have only three vertices and all of them are among the patch vertices.
% % %                 patch_handle = patch('Faces',faces,'Vertices',verts,'FaceColor','g','EdgeColor','r');
% % %                 feedback_plot_handle = [feedback_plot_handle, patch_handle]; %#ok<AGROW>
% % %                 
% % %                 % in the following, we write the number of the node on the
% % %                 % corresponding parallel lines.
% % %                 text_dist = shiftet_dist/2;
% % %                 text_position = bottom_vertices_outer + text_dist*perpendicular_vector_normalized;
% % %                 text_position(1) = text_position(1) - 0.45; % for some reason MATLAB shifts the starting point of the text a little bit to the right. So, here we return it back.
% % %                 if feedback_text_handle (edge_num) ~= 0 % which means some text has already been written for this edge
% % %                     current_text = get(feedback_text_handle (edge_num), 'String');
% % %                     set(feedback_text_handle (edge_num), 'String', [current_text, ', ', num2str(i)])
% % %                 else
% % %                     feedback_text_handle (edge_num) = text(text_position(1), text_position(2), num2str(i), 'fontsize',10,'color','r','EdgeColor','g');
% % %                     feedback_plot_handle = [feedback_plot_handle, feedback_text_handle (edge_num)]; %#ok<AGROW>
% % %                 end
% % %             end
        end
        function nearest_node_ind = compute_nearest_node_ind(obj,current_node_ind)
            % This function computes the nearest node to the "current_node_ind"
            neighbors = find(obj.edges_matrix(current_node_ind,:));
            weighted_norm = nan(1,length(neighbors));
            
            for i = 1 : length(neighbors)
                elem_wise_distance = abs(obj.nodes(current_node_ind).signed_element_wise_dist(obj.nodes(neighbors(i)))); % Never forget "abs" function
                weighted_norm(i) = norm(elem_wise_distance .* state.sup_norm_weights , 2); % last argument tells that which norm you are using. Here for PRM we use 2-norm.
            end
            [~,min_ind] = min(weighted_norm);
            nearest_node_ind = neighbors(min_ind);
        end
    end
    
    methods (Access = private)
        function neighbors = find_neighbors(obj,i)
            neighbors = [];
            for j = [1:i-1,i+1:obj.num_nodes]
                elem_wise_distance = abs(obj.nodes(i).signed_element_wise_dist(obj.nodes(j))); % Never forget "abs" function
                % disp('the following line is only true in multi-robot')
                % if ~all(elem_wise_distance), continue; end
                weighted_norm = norm(elem_wise_distance .* state.sup_norm_weights , 2); % last argument tells that which norm you are using. Here for PRM we use 2-norm.
                if weighted_norm < obj.par.neighboring_distance_threshold
                    neighbors = [neighbors,j]; %#ok<AGROW>
                end
            end
        end
        function obj = draw_edge(obj,edge_number)
            traj_flag = 0; % this has to be fixed. This should be defined by user
            tmp_handle = MotionModel_class.draw_nominal_traj(obj.edges(edge_number), traj_flag);
            obj.edges_plot_handle = [obj.edges_plot_handle,tmp_handle];
        end
    end
end