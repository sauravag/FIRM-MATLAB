classdef PRM_class < PRM_interface
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
        function obj = PRM_class(~)
            % See the superclass; The constructor of the superclass, i.e., "PRM_interface" is
            % automatically called.
        end
        function obj = draw(obj)
            % This function draws the PRM graph. If no "plot properties"
            % are specified by the user, the default "PRM plot properties"
            % will be used.
            old_prop = obj.set_figure();
            % The following initializations are necessary. Because the initialization in the "property definition" is not enough, when we loading an existing object of this class, that initialization does not happen.
<<<<<<< HEAD
            obj.edges_plot_handle = [];
            % retrieve PRM parameters provided by the user
            node_text_flag = obj.par.PRM_node_text;
            varargin_node_props = obj.par.PRM_node_plot_properties;
            % some variable definitions
            N_nodes=obj.num_nodes;
            % draw edges
            for i = 1:size(obj.edges_list , 1)
                %obj = obj.draw_edge(i);
            end
            % Draw Nodes
            for p=1:N_nodes
                if  node_text_flag == 1
                    obj.nodes(p) = obj.nodes(p).draw(varargin_node_props{:},'text',num2str(p));
                else
                    obj.nodes(p) = obj.nodes(p).draw(varargin_node_props{:});
=======
            obj.orbit_edges_traj_handle = [];
            obj.orbit_edges_plot_handle = [];
            obj.orbit_text_handle = [];
            
            %             text_size = obj.par.orbit_text_size;  % User-provided value for "OrbitTextSize" property.
            %             text_color = obj.par.orbit_text_color; % User-provided value for "OrbitTextColor" property.
            %             text_shift = obj.par.orbit_text_shift; % User-provided value for shifting the text a little bit to the left. % for some reason MATLAB shifts the starting point of the text a little bit to the right. So, here we return it back.
            %             robot_shape = obj.par.orbit_robot_shape;
            %robot_size = obj.par.orbit_robot_size;
            node_to_orbit_trajectories_flag = obj.par.node_to_orbit_trajectories_flag;
            %             orbit_edge_spec = obj.par.orbit_edge_spec;
            %             orbit_edge_width = obj.par.orbit_edge_width;
            
            % drawing orbits and orbit numbers
            for i = 1:obj.num_orbits
                orbit_plot_varargin = {'OrbitText', num2str(i)};
                obj.orbits(i) = MotionModel_class.draw_orbit(obj.orbits(i), orbit_plot_varargin{:});  % the plot_handle of the orbit is saved as a filed of orbit itself.
            end
            
            % drawing orbit_edges
            for i = 1 : size(obj.orbit_edges_trajectory,1)
                for j = 1 : size(obj.orbit_edges_trajectory,2)
                    if ~isempty(obj.orbit_edges_trajectory(i,j).x)
                        traj_plot_handle = MotionModel_class.draw_nominal_traj(obj.orbit_edges_trajectory(i,j), node_to_orbit_trajectories_flag);
                    end
>>>>>>> ba6fc21e4458b0b1914888aaba4114631ed30a9f
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
            error('This function is obsolete. But it can be updated beautifully with the new was of looking to feedback!')
            % This function has not been updated after the latest changes.
            
            % the selected nodes are a set of nodes that the feedback pi is
            % only drawn for them. This is for uncluttering the figure.
            if ~exist('selected_node_indices', 'var') % if there is no "selected nodes", we draw the "feedback pi" for all nodes.
                selected_node_indices = 1:obj.num_nodes;
            elseif isvector(selected_node_indices) % the "selected_node_indices" has to be a row vector. So, here we make the code robust to column vector as well.
                selected_node_indices = reshape(selected_node_indices,1,length(selected_node_indices));
            end
            feedback_plot_handle = [];
            num_edges = length(obj.edges_list);
            feedback_text_handle = zeros(1,num_edges);
            for i = selected_node_indices
                start = obj.nodes(i).val(1:2); % from now on, in this function, we only consider 2D position of the nodes.
                j = feedback_pi(i); % j is the next node for node i, based on "feedback pi".
                if isnan(j) % the feedback_pi on the goal node return "nan"
                    continue
                end
                [~,edge_num] = intersect(obj.edges_list,[i,j],'rows'); % this function returns the number of PRM edge, whose start and end nodes are i and j, respectively.
                edge_num =obj.corresponding_2D_edges(edge_num); % this line returns the number of corresponding 2D edge.
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
<<<<<<< HEAD
        function neighbors = find_neighbors(obj,i)
=======
        function obj = request_orbits(obj)
            % This function reveives orbits from the user and draws them.
            old_prop = obj.set_figure();
            title({'Please mark the center of orbits'},'fontsize',14)
            tmp_neighb_plot_handle = [];
            scale = obj.par.neighboring_distance_threshold;
            for i = 1:obj.max_number_of_orbits
                new_orbit = MotionModel_class.sample_a_valid_orbit();
                if isempty(new_orbit)
                    delete(tmp_neighb_plot_handle)
                    break
                else
                    new_orbit = obj.select_nodes_on_an_orbit(new_orbit);
                    obj = obj.add_set_of_orbits(new_orbit);
                    % depict the distance from the node that can connect to
                    % other nodes
                    delete(tmp_neighb_plot_handle)
                    %tmp_neighb_plot_handle = MotionModel_class.draw_orbit_neighborhood(new_orbit, scale);
                    disp('uncomment line 273 orbit_prm_class for the drawing');
                end
            end
            obj.reset_figure(old_prop)
            title([])
            obj.num_orbits = size(obj.orbits,2);
        end
        function orbit = select_nodes_on_an_orbit(obj, orbit)
            % defining PNPRM nodes on the orbits
            orbit.num_nodes = obj.par.num_nodes_on_orbits; % number of nodes on each orbit
            time_dist = orbit.period/orbit.num_nodes; % distance between two consecutive nodes on an orbit. (here, it is a float number)
            if time_dist < 1, error('The number of nodes on orbit is more than the length of orbit.'), end
            time_dist = floor(time_dist); % distance between two consecutive nodes on an orbit. (Now, it is a whole number)
            s = ceil(orbit.period/2); % we start from the middle of orbit
            tmp_locs = s+[0:orbit.num_nodes-1]*time_dist; %#ok<NBRAK> % temporary node locations (time steps) on an orbit
            orbit.node_time_stages = mod(tmp_locs,orbit.period)+(orbit.period*(mod(tmp_locs,orbit.period)==0)); % Here, we apply the periodic nature of the orbit to keep the node time between 1 and T.
        end
        function obj = add_orbit(obj,new_orbit)
            % first we increase the number of orbits by one
            obj.num_orbits = obj.num_orbits + 1;
            % assign an index to the new orbit
            new_orbit_ind = obj.num_orbits;
            disp(['Adding PNPRM orbit ',num2str(new_orbit_ind), ' to the PNPRM']);
            % add it to the list of orbits
            if isempty(obj.orbits)  % I put this "if statement" becuase if "obj.orbits" is empty, Matlab does not let you to save the "new_orbit" in its first entry.
                obj.orbits = new_orbit;
            else
                obj.orbits(new_orbit_ind) = new_orbit;
            end
            % drawing the new node
            orbit_prop_varargin = {}; %obj.par.PRM_orbit_plot_properties;
%             if ~isfield(new_orbit, 'plot_handle') || isempty(new_orbit.plot_handle)  % check if the orbit is already drawn
%                 obj.orbits(new_orbit_ind) = MotionModel_class.draw_orbit(obj.orbits(new_orbit_ind), orbit_prop_varargin{:});
%             end
           disp('Uncomment the above three lines for drawing Orbit_PRM_Class.m line 307');
            % we compute the neighbors of the newly added orbit.
            neighbors_of_new = obj.find_orbit_neighbors(new_orbit_ind);
            % we add the orbit_edges corresponding to this new orbit. 
            for j = neighbors_of_new % "neighbors_of_new" must be a row vector for this loop to work.
                % We first consider the orbit_edges that goes out of this orbit
                obj = add_orbit_edge(obj, new_orbit, obj.orbits(j), new_orbit_ind, j);
                % Then, we consider the orbit_edges that comes into this orbit.
                obj = add_orbit_edge(obj, obj.orbits(j), new_orbit, j, new_orbit_ind);
            end
            % since "obj.orbit_edges_matrix" may not be square after above
            % opertations (if the reverse orbit_edges are not collision-free), we make it square here.
            tmp = zeros(obj.num_orbits,obj.num_orbits);
            tmp(1:size(obj.orbit_edges_matrix,1) , 1:size(obj.orbit_edges_matrix,2)) = obj.orbit_edges_matrix;
            obj.orbit_edges_matrix = tmp;
        end
        function obj = add_orbit_edge(obj, start_orbit, end_orbit, start_orbit_ind, end_orbit_ind)
            nominal_traj = MotionModel_class.generate_VALID_open_loop_orbit2orbit_traj(start_orbit, end_orbit); % generates open-loop trajectories between two start and end orbits
            if ~isempty(nominal_traj)
                obj.orbit_edges_list = [obj.orbit_edges_list ; [start_orbit_ind , end_orbit_ind]]; % adding orbit_edge to the list of orbit_edges
                if isempty(obj.orbit_edges_trajectory) % I put this "if statement" becuase if "obj.orbit_edges_trajectory" is empty, Matlab does not let you to save the "nominal_traj" in any of its entries.
                    obj.orbit_edges_trajectory.x = [];obj.orbit_edges_trajectory.u = [];
                end
                obj.orbit_edges_trajectory(start_orbit_ind , end_orbit_ind) = nominal_traj; % adding orbit itself to the set of orbits
                traj_plot_handle = MotionModel_class.draw_nominal_traj(nominal_traj, obj.par.node_to_orbit_trajectories_flag); % plot the orbit
                obj.orbit_edges_plot_handle = [obj.orbit_edges_plot_handle , traj_plot_handle];
                %disp('Uncomment line 332/3 Orbit_PRM_class for drawing');
                obj.orbit_edges_matrix(start_orbit_ind , end_orbit_ind) = 1;
            end
        end
        function obj = construct_nodes(obj)
            obj.nodes = state.empty; % class type initialization
            n = 0; % n represents the absolute number of PRM nodes (Not on a single orbit, but among all PRM nodes)
            for i = 1:obj.num_orbits
                for alpha = 1:obj.orbits(i).num_nodes
                    n = n+1;
                    node_time_on_orbit = obj.orbits(i).node_time_stages(alpha);
                    node_tmp(alpha) = state ( obj.orbits(i).x(:,node_time_on_orbit) ); %#ok<AGROW>
                    obj.nodes(n) = node_tmp(alpha);
                    obj.corresponding_orbit(n) = i;
                end
            end
            obj.num_nodes = n;
        end
        function obj = construct_edges(obj)
            error('The name and inside of this function needs to be changed as we differentiate between orbit_edges and edges')
            obj.edges_matrix = obj.orbit_edges_matrix;
            obj.edges_list = obj.orbit_edges_list;
        end
        function neighbors = find_orbit_neighbors(obj,i_orbit)
>>>>>>> ba6fc21e4458b0b1914888aaba4114631ed30a9f
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