classdef PNPRM_class < PRM_interface
    
    properties
        num_orbits = 0; % this variable needs to be initialized to zero here.
        orbits;
        orbit_edges_trajectory;  % The edge trajectories that connect orbits to orbits
        corresponding_orbit; % i-th element of this property is the number of orbit on which the i-th node lies.
        orbit_edges_traj_handle = [];
    end
    
    properties (Access = private)
        edges_traj_handle = []; % plot handle for edge trajectories (note that edge trajectory can be different from the edge itself, whose plot handle is "edges_plot_handle", due to the discretization errors)
        num_2Dnodes;
        nodes_2D;
        orbit_edges_list = [];
        orbit_edges_matrix;
        max_number_of_orbits = 50;
    end
    
    properties (Access = private)
        % following properties are defined as the abstract properties in the
        % supperclass. However, since in Matlab private properties are not
        % inherited, we have have to rewrite them consistently in all
        % subclasses.
        edges_plot_handle = [];
        orbit_text_handle = [];
    end
    
    methods
        function obj = PNPRM_class(~)
            % The constructor of the superclass, i.e., "PRM_interface" is
            % automatically called.
        end
        function obj = draw(obj)
            old_prop = obj.set_figure();
            % The following initializations are necessary. Because the initialization in the "property definition" is not enough, when we loading an existing object of this class, that initialization does not happen.
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
                        traj_plot_handle = MotionModel_class. draw_nominal_traj(obj.orbit_edges_trajectory(i,j), node_to_orbit_trajectories_flag);
                    end
                end
            end
            
            obj.reset_figure(old_prop);
        end
        function obj = delete_plot(obj)
            error('Ali:')
            delete(obj.orbit_edges_traj_handle)
            obj.orbit_edges_traj_handle = [];
            for i = 1:obj.num_orbits
                delete(obj.orbits(i).plot_handle)
                obj.orbits(i).plot_handle = [];
            end
            delete(obj.orbit_text_handle);
            obj.orbit_text_handle = [];
            for p = 1:length(obj.nodes)
                obj.nodes(p) = obj.nodes(p).delete();
            end
            delete(obj.orbit_edges_plot_handle);
            obj.orbit_edges_plot_handle = [];
        end
        function obj = load(obj) %#ok<MANU>
            % This function loads the existing PNPRM from the
            % "LoadFileName". If there is no PNPRM present in the
            % "LoadFileName", an error will occur.
            LoadFileName = user_data_class.par.LoadFileName;
            load(LoadFileName,'PNPRM')
            obj = PNPRM;
        end
        function obj = request_nodes(obj)
            obj = obj.request_orbits();  % This function reveives orbits from the user and draws them.
            obj = obj.construct_nodes();
            obj = obj.construct_orbit_edges();
        end
        function obj = overwrite_nodes(obj)
        end
        function obj = save(obj)
            SaveFileName = user_data_class.par.SaveFileName;
            PNPRM = obj; %#ok<NASGU>
            save(SaveFileName,'PNPRM','-append')
        end
        
        function obj = add_node(obj,new_node)
            error('This function has not been implemented for PNPRM yet')
        end        
        function obj = add_set_of_orbits(obj,new_orbits)
            for i = 1:length(new_orbits)
                obj = obj.add_orbit(new_orbits(i));
            end
        end
        function YesNo = has_same_orbit(obj,i,j)
            error('Ali:')
            % this function returns 1 if the corresponding orbit of the nodes
            % "i" and "j" are same. And it returns 0 otherwise.
            if (obj.corresponding_orbit(i) == obj.corresponding_orbit(j))
                YesNo = 1;
            else
                YesNo = 0;
            end
        end
        function feedback_plot_handle = draw_feedback_pi(obj, feedback_pi, selected_node_indices)
            error('This function has not been updated from PRM to PNPRM, yet.')
            % the selected nodes are a set of nodes that the feedback pi is
            % only drawn for them. This is for uncluttering the figure.
            if ~exist('selected_node_indices', 'var') % if there is no "selected nodes", we draw the "feedback pi" for all nodes.
                selected_node_indices = 1:obj.num_nodes;
            elseif isvector(selected_node_indices) % the "selected_node_indices" has to be a row vector. So, here we make the code robust to column vector as well.
                selected_node_indices = reshape(selected_node_indices,1,length(selected_node_indices));
            end
            feedback_plot_handle = [];
            num_2Dorbit_edges = length(obj.orbit_edges_2D_list);
            feedback_text_handle = zeros(1,num_2Dorbit_edges);
            for i = selected_node_indices
                start = obj.nodes(i).val(1:2); % from now on, in this function, we only consider 2D position of the nodes.
                j = feedback_pi(i); % j is the next node for node i, based on "feedback pi".
                if isnan(j) % the feedback_pi on the goal node return "nan"
                    continue
                end
                [~,orbit_edge_num] = intersect(obj.orbit_edges_list,[i,j],'rows'); % this function returns the number of orbit_edge, whose start and end nodes are i and j, respectively.
                orbit_edge_num =obj.corresponding_2D_orbit_edges(orbit_edge_num); % this line returns the number of corresponding 2D orbit_edge.
                % in the following we draw a paraller line to the orbit_edge,
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
                if feedback_text_handle (orbit_edge_num) ~= 0 % which means some text has already been written for this orbit_edge
                    current_text = get(feedback_text_handle (orbit_edge_num), 'String');
                    set(feedback_text_handle (orbit_edge_num), 'String', [current_text, ', ', num2str(i)])
                else
                    feedback_text_handle (orbit_edge_num) = text(text_position(1), text_position(2), num2str(i), 'fontsize',10,'color','r','EdgeColor','g');
                    feedback_plot_handle = [feedback_plot_handle, feedback_text_handle (orbit_edge_num)]; %#ok<AGROW>
                end
            end
        end
        function nearest_orbit_ind = compute_nearest_orbit_ind(obj,current_node_ind)
            error('This function has not been updated from PRM to PNPRM, yet.')
            % This function computes the nearest orbit to a given ???
            neighbors = find(obj.orbit_edges_matrix(current_node_ind,:));
            dist = nan(1,length(neighbors));
            for i = 1 : length(neighbors)
                dist(i) = norm( obj.nodes(current_node_ind).val(1:2) - obj.nodes(neighbors(i)).val(1:2) );
            end
            [~,min_ind] = min(dist);
            nearest_node_ind = neighbors(min_ind);
        end
        function nominal_traj = generate_node_to_orbit_trajectory(obj , start_orbit_ind, node_ind_on_orbit , end_orbit_ind)
            % This funtion concatenates a part on orbit and the "PRM.orbit_edges_trajectory" to generate the node to
            % orbit trajectories.
                
            % computing closest state on the orbit to the
            % "temp_orbit_edge_start" %  Not complete
            start_orbit = obj.orbits(start_orbit_ind);
            end_orbit = obj.orbits(end_orbit_ind);
            
            alpha = node_ind_on_orbit; % the number of node on the starting orbit.
            node_time_stage = start_orbit.node_time_stages(alpha); % the time stage of alpha-th node on i-th orbit
            %initial_gamma = atan2(start_orbit.x(2,1) - start_orbit.center(2),start_orbit.x(1,1) - start_orbit.center(1));  % The anlge on which the start of orbit lies.
            node_gamma   = atan2(start_orbit.x(2,node_time_stage) - start_orbit.center(2) , start_orbit.x(1,node_time_stage) - start_orbit.center(1));  % The anlge on which the "alpha"-th node on orbit lies.
            angle_of_connecting_line = atan2( end_orbit.center(2) - start_orbit.center(2) , end_orbit.center(1) - start_orbit.center(1) ); % the angle of the orbit_edge connecting two orbit i to j
            gamma_start_of_orbit_edge = angle_of_connecting_line - pi/2; % the angle on which the starting point of orbit_edge lies on orbit i.
            
            % making "gamma" and "node_gamma" positive.
            % it is very problematic part (verified by plotting)
            gamma_start_of_orbit_edge = mod(gamma_start_of_orbit_edge, 2*pi);
            node_gamma = mod(node_gamma, 2*pi);
            if node_gamma>gamma_start_of_orbit_edge
                node_gamma = node_gamma - 2*pi;
            end
            % till here
            
            delta_theta_on_orbit = 2*pi/start_orbit.period;
            steps_till_start_of_orbit_edge = floor((gamma_start_of_orbit_edge - node_gamma)/delta_theta_on_orbit);  % This "floor" can make a lot of problems. You have to take it out ASAP.
            error(' the above line is fixed in the unicycle class, correct it please')
            steps_till_start_of_orbit_edge = mod(steps_till_start_of_orbit_edge,start_orbit.period);
            T = start_orbit.period;
            more_than_period = node_time_stage+steps_till_start_of_orbit_edge - T;
            if more_than_period < 0
                more_than_period = 0;
            end
            pre_edge_traj.x = [ start_orbit.x(:,node_time_stage : node_time_stage+steps_till_start_of_orbit_edge - more_than_period) , start_orbit.x(:,1:more_than_period) ];
            pre_edge_traj.u = [start_orbit.u(:,node_time_stage : node_time_stage+steps_till_start_of_orbit_edge - more_than_period -1) , start_orbit.u(:,1:more_than_period)];
            
            % making "gamma_end_of_orbit_edge" positive.
            gamma_end_of_orbit_edge = gamma_start_of_orbit_edge; % this is correct when the conncting lines (from i to j and from j to i) does not interesect with each other. i.e., when they are parallel.
            initial_gamma = 3*pi/2;
            angle_diff = gamma_end_of_orbit_edge - initial_gamma;
            if angle_diff <0
                angle_diff = angle_diff +2*pi;
            end
            
            end_orbit_edge_time = floor((angle_diff)/delta_theta_on_orbit);
            if end_orbit_edge_time ==0, end_orbit_edge_time =T; end
            post_edge_traj.x = [ end_orbit.x(:,end_orbit_edge_time:T) ];
            post_edge_traj.u = [ end_orbit.u(:,end_orbit_edge_time:T) ];
            
            nominal_traj.x = [pre_edge_traj.x , obj.orbit_edges_trajectory(start_orbit_ind,end_orbit_ind).x , post_edge_traj.x];
            nominal_traj.u = [pre_edge_traj.u , obj.orbit_edges_trajectory(start_orbit_ind,end_orbit_ind).u , post_edge_traj.u];
            
            disp('Following lines seems problematic')
            nominal_traj.x = [pre_edge_traj.x(:,1:end-1) , obj.orbit_edges_trajectory(start_orbit_ind,end_orbit_ind).x(:,1:end-1) , post_edge_traj.x];
            nominal_traj.u = [pre_edge_traj.u , obj.orbit_edges_trajectory(start_orbit_ind,end_orbit_ind).u , post_edge_traj.u];
            
        end
    end
    
    methods (Access = private)
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
                    tmp_neighb_plot_handle = MotionModel_class.draw_orbit_neighborhood(new_orbit, scale);
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
            if ~isfield(new_orbit, 'plot_handle') || isempty(new_orbit.plot_handle)  % check if the orbit is already drawn
                obj.orbits(new_orbit_ind) = MotionModel_class.draw_orbit(obj.orbits(new_orbit_ind), orbit_prop_varargin{:});
            end
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
            nominal_traj = MotionModel_class.generate_open_loop_orbit2orbit_traj(start_orbit, end_orbit); % generates open-loop trajectories between two start and end orbits
            if ~isempty(nominal_traj)
                obj.orbit_edges_list = [obj.orbit_edges_list ; [start_orbit_ind , end_orbit_ind]]; % adding orbit_edge to the list of orbit_edges
                if isempty(obj.orbit_edges_trajectory) % I put this "if statement" becuase if "obj.orbit_edges_trajectory" is empty, Matlab does not let you to save the "nominal_traj" in any of its entries.
                    obj.orbit_edges_trajectory.x = [];obj.orbit_edges_trajectory.u = [];
                end
                obj.orbit_edges_trajectory(start_orbit_ind , end_orbit_ind) = nominal_traj; % adding orbit itself to the set of orbits
                traj_plot_handle = MotionModel_class.draw_nominal_traj(nominal_traj, obj.par.node_to_orbit_trajectories_flag); % plot the orbit
                obj.orbit_edges_plot_handle = [obj.orbit_edges_plot_handle , traj_plot_handle];
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
            error('The name and inside of this function needs to be changes as we differentiate between orbit_edges and edges')
            obj.edges_matrix = obj.orbit_edges_matrix;
            obj.edges_list = obj.orbit_edges_list;
        end
        function neighbors = find_orbit_neighbors(obj,i_orbit)
            neighbors = [];
            for j_orbit = [1:i_orbit - 1 , i_orbit+1:obj.num_orbits]
                if norm(obj.orbits(i_orbit).center - obj.orbits( j_orbit ).center) < ...
                        obj.par.neighboring_distance_threshold
                    neighbors = [ neighbors,j_orbit ]; %#ok<AGROW>
                end
            end
        end
    end
end