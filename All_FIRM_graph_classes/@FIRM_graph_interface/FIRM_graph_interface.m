classdef FIRM_graph_interface
    %FIRM_interface is the base class, from which different variants of the FIRM frameworks is derived.
    
    properties
        PRM; % underlying PRM graph
        Stabilizers; % stabilizers in the FIRM framework.
        Nodes;  % FIRM nodes
        Edges;  % FIRM edges
        num_stabilizers; % number of stabilizers in FIRM framework
        num_nodes; % number of FIRM nodes
        num_edges; % number of FIRM edges
        cost_to_go; % cost to go vector, whose i-th element is the cost-to-go associated with i-th FIRM node
        feedback_pi; % feedback vector, whose i-th element is the optimal local controller (next edge) associated with i-th FIRM node
        plot_handle_feedback_pi % the "plot handle" for the arrows that represent the feedback "pi" on the underlying PRM graph
    end
    
    methods
        function obj = FIRM_graph_interface(PRM_inp)
            % constructor of the FIRM interface
            obj.PRM = PRM_inp;
            obj.num_nodes = PRM_inp.num_nodes;
        end
        function obj = draw_all_nodes(obj)
            for i=1:obj.num_nodes
                obj.Nodes(i) = obj.Nodes(i).draw();  % Besides drawing the node, we update the  "obj.Nodes(i).node.plot_handle" here in this line.
            end
        end
         function obj = delete_all_nodes(obj)
            for i=1:obj.num_nodes
                obj.Nodes(i) = obj.Nodes(i).delete_plot();  % Besides drawing the node, we update the  "obj.Nodes(i).node.plot_handle" here in this line.
            end
        end
        function obj = construct_all_FIRM_edges(obj)
            % This function constructs FIRM edges and assigns values to the "edges" property of the class.
            for i = 1:obj.num_edges
                tic
                disp(['Constructing edge ',num2str(i),' ...'])
                start_ind = obj.PRM.edges_list(i,1);
                end_ind = obj.PRM.edges_list(i,2);
                % The following funtion concatenates a part on orbit and
                % the "PRM.orbit_edges_trajectory" to generate the node to
                % orbit trajectories.
                PRM_edge_traj = obj.PRM.generate_node_to_orbit_trajectory(start_ind,end_ind); % the end_ind has to be orbit. But since right now we only have a single node on each orbit, they are the same.
                % To test correcness
                for iii=1:length(PRM_edge_traj.x)
                    xp=state(PRM_edge_traj.x(:,iii));
                    xp.draw('RobotShape','triangle','triacolor','g','color','g');
                end
                
                obj.Edges(i) = FIRM_edge_class(obj.Nodes(start_ind),obj.Nodes(end_ind) , i , PRM_edge_traj ); % the last input has been added on 01/29/2012
                obj.Edges(i) = obj.Edges(i).construct();
                obj.time_of_edge_construction(i) = toc;
            end
        end
        function obj = DP_compute_cost_to_go_values(obj,goal_node_ind)
            disp('We should bring the constants in this function to the "user_data_class".')
            values = zeros(obj.num_nodes,1); % I can consider one more value here for failure state. However, I am going to handle the failure state in a different way as it is seen in below.
            feedback_solution = nan(obj.num_nodes,1); % feedback_solution(i) is the output of "feedback pi", when we are at i-th node.
            list_of_failure_prob = [obj.Edges.failure_probability]; % This is the list of failure probabilities corresponding to every edge. Note that the "brackets" at the right hand side play an important role here.
            list_of_total_costs = -[obj.Edges.FIRM_cost]; % note that since we are doing "maximization", actually these are "rewards" NOT "costs". Thus, we have to make them negative.
            J_fail = - user_data_class.par.failure_cost_to_go; %failure cost-to-go % similar to above line, this cost is also has to be negative.
            
            value_mat = []; % just for monitoring value updates. Otherwise, not useful.
            new_values = ones(obj.num_nodes,1)*user_data_class.par.initial_values; % initializing node values.
            while norm(values-new_values)>user_data_class.par.DP_convergence_threshold
                values = new_values;
                value_mat = [value_mat,values]; %#ok<AGROW> % just for monitoring value updates. Otherwise, not useful.
                for i = 1 : size(values,1) % i  is the node number (absolute number)
                    if i == goal_node_ind % The value of the goal node must remain unchanged.
                        new_values(i) = user_data_class.par.initial_value_goal;
                        feedback_solution(i) = nan;
                    else
                        % s_prime = find(obj.PRM.edges_matrix(i,:)); % s_prime is the list of nodes that i-th node is connected to.
                        % possible_s_prime = obj.Edges(i).possible_end_nodes; % "possible_s_prime" is the list of nodes that i-th node is connected to.
                        
                        candidate_values = []; % initialization
                        available_edges = obj.Nodes(i).outgoing_edges; % this is the available edges (local controllers) that can be invoked from node i
                        for edge_num = available_edges  % NOTE that in this kind of indexing the right hand side of equality sign must be a row vector
                            
%                             tmp_sum = 0;
                            tmp_ind = 0;
                            Cij = list_of_total_costs(edge_num); % cost of transition from node i to node j % NOTE that Cij can be a function of j. So, we have to keep it in the loop for generality.
%                             Cfail = list_of_total_costs(edge_num); % if the target node is "failure node", we still consider the transition cost, the same as the case that the target node is Not failure. And we apply the "undesirability" of the failure node by assigning a high cost-to-go value for it.
                            % Note that in general by taking action j, there are some probabilities
                            % that you land in different states (Here that
                            % probability is zero). The following for loop,
                            % loops over those possible destination nodes.
                            possible_next_nodes = obj.Edges(edge_num).possible_end_node_indices;
                            if any(i == possible_next_nodes)
                                error('we do not let staying in the same state.'); % Or we need to do something like this: Cij = Cij + (-1000); % We add 1000 to the cost, so that robot does not stay in the current node. % Again it has to be negative.
                            end
                            Pij_vector = [];
                            value_j_vector = [];
                            for j = possible_next_nodes  % j is the next node number (absolute number)
                                tmp_ind = tmp_ind +1;
                                Pij = obj.Edges(edge_num).reaching_probabilities(tmp_ind);
                                Pij_vector = [Pij_vector , Pij]; %#ok<AGROW>
                                value_j_vector = [value_j_vector, values(j)]; %#ok<AGROW>
%                                 tmp_sum = tmp_sum + Pij*(Cij + values(j));
                            end
                            P_fail = list_of_failure_prob(edge_num); % transition probability from node i to node j
                            full_P_vector = [Pij_vector , P_fail];
                            full_value_vector = [value_j_vector , J_fail];
                            candidate_values = [candidate_values, sum(full_P_vector(1:end-1))*Cij+full_P_vector*full_value_vector' ]; %#ok<AGROW>
                        end
                        if isempty(candidate_values)
                            new_values(i) = 0;
                            feedback_solution(i) = nan;
                            disp('Make sure this line is Ok in the DP solver');
                        else
                            [new_values(i),best_sp_ind] = max(candidate_values);
                            feedback_solution(i) = available_edges(best_sp_ind);  % assigns the number of best edge (local controller) that has to be invoked from node i.
                        end
                    end
                end
            end
            %             % In the following we draw the evolution of node values.
%             curr_fig = gcf;
%             figure;
%             plot(value_mat')
%             figure(curr_fig);
%             drawnow
            % in the following we draw the arrows on the
            % storing the values and "feedback_pi" in the graph.
            obj.cost_to_go = values;
            obj.feedback_pi = feedback_solution;
            % in the following we plot the feedback pi on the graph.
            selected_nodes = user_data_class.par.selected_nodes_for_plotting_feedback_pi;
%             if ~isempty(selected_nodes )
%             obj.plot_handle_feedback_pi = obj.PRM.draw_feedback_pi(obj.feedback_pi, obj.Edges, selected_nodes);
%             end
        end
    end
    
    methods (Access = private)
        function obj = add_a_node_and_its_sequel(obj,b)
            % HBRM node
            PRM_new_node = b.est_mean;
            new_node_ind = obj.num_nodes + 1;
            obj.Nodes(new_node_ind) = FIRM_node_class(PRM_new_node,new_node_ind);
            % inserting singleton GHb to the node
            singleton_GHb = Hbelief_G(b.est_mean,b.est_mean,b.est_cov,blkdiag(b.est_cov,zeros(state.dim)));
            obj.Nodes(new_node_ind).stationary_GHb = singleton_GHb;
            % Important: Note that the "singleto_GHb", which is inserted as
            % the "stationary_GHb" of the new node, is only used to produce
            % the new edges at this stage. After producing the edges, we
            % UPDATE the "stationary_GHb" by its real "stationary_GHb".
            
            % set the new number of nodes by one
            obj.num_nodes = obj.PRM.num_nodes;
            % draw the new nodes
            obj.Nodes(new_node_ind).stationary_GHb = obj.Nodes(new_node_ind).stationary_GHb.draw();
            % new edges
            old_num_of_edges = obj.num_edges;
            obj.num_edges = size(obj.PRM.edges_list,1);
            for i = old_num_of_edges+1 : obj.num_edges
                tic
                disp(['Constructing edge ',num2str(i),' ...'])
                start_ind = obj.PRM.edges_list(i,1);
                end_ind = obj.PRM.edges_list(i,2);
                obj.Edges(i) = FIRM_edge_class(obj.Nodes(start_ind),obj.Nodes(end_ind),i);
                if ~user_data_class.par.goBack_to_nearest_node % if, in replanning, we go to the nearest node, we do not need to compute the uncertainty porpagation along the newly added edges.
                    obj.Edges(i) = obj.Edges(i).construct();
                end
                obj.time_of_edge_construction(i) = toc;
            end
            
            % In the following line we compute the real stationary GHb
            % corresponding to the new node. Note that this has to be done
            % after computing edges. 
            % Note that the reason we have to update this "stationary_GHb"
            % from "singleton" value to this value, is that this newly
            % added node can be the "end node" of some other edges which
            % may be added later.
            disp('Following is wrong!! because, if you want to use the added node again in the planning, the costs has to be computed based on the stationary GHb, not the singleton GHb.')
            obj.Nodes(new_node_ind) = obj.Nodes(new_node_ind).construct_node();
            obj.Nodes(new_node_ind).stationary_GHb = obj.Nodes(new_node_ind).stationary_GHb.draw();
        end
    end

    methods (Abstract)
        obj = construct_all_stabilizers_and_FIRM_nodes(obj)
        obj = Execute(obj,initial_Hstate,start_node_ind,goal_node_ind)
    end
    
end