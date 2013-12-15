classdef PLQG_based_FIRM_graph_class < FIRM_graph_interface
    %PLQG_based_FIRM_graph_class is the FIRM class based on stationary LQG controllers.
    
    properties (Access = private)
        time_of_edge_construction; % time it takes the algorithm to construct and edge along with its collision probabilities and costs.
    end
    
    methods
        function obj = PLQG_based_FIRM_graph_class(PRM_inp)
            obj = obj@FIRM_graph_interface(PRM_inp);
            obj.num_stabilizers = obj.PRM.num_orbits;
            obj.num_edges = size( obj.PRM.edges_list,1) * obj.PRM.orbits(1).num_nodes;
            
            obj.Stabilizers = stabilizer_class.empty;
            obj.Stabilizers(obj.num_stabilizers,1) = stabilizer_class; % Preallocate object array
            obj.Nodes = FIRM_node_class.empty;
            obj.Nodes(obj.num_nodes,1) = FIRM_node_class; % Preallocate object array
            obj.Edges = FIRM_edge_class.empty;
%             obj.Edges(obj.num_edges,1) = FIRM_edge_class; % Preallocate object array
        end
        function obj = construct_all_stabilizers_and_FIRM_nodes(obj)
            % This function constructs stabilizers used in PLQG-based FIRM framework and constructs reachable nodes under this stabilizers and assigns values to the "stabilizers" and "nodes" properties of the class.
            num_of_nodes = obj.PRM.num_nodes;
            obj.num_nodes = num_of_nodes;
            num_oribts = obj.PRM.num_orbits;
            tic
            % we design the stabilizers (belief orbit stabilizers) in the following loop.
            for i = 1:obj.num_stabilizers % i is the stabilizer number (or orbit number in this class)
                nodes_on_orbit = (obj.PRM.corresponding_orbit == i);
                PRM_nodes_on_orbit = obj.PRM.nodes(nodes_on_orbit);  % The set of PRM nodes on orbit  i
                target_node_indices = [];
                for j =1:length(nodes_on_orbit)
                    if nodes_on_orbit(j)==1
                        target_node_indices= [target_node_indices,j];
                    end
                end
                
                PRM_orbit = obj.PRM.orbits(i);
                disp(['Orbit Stabilizer ',num2str(i),' out of total ',num2str(obj.num_stabilizers),' stabilizers'])
                obj.Stabilizers(i).stabilizer_number = i;  % We need this for display purposes in the "Orbit_stabilizer_PLQG_class". However, we do not provide it as a constructor input, since it is not a real property of the class.
                obj.Stabilizers(i) = stabilizer_class(PRM_nodes_on_orbit  ,  PRM_orbit, target_node_indices);  % constructing i-th stabilizer
            end
            n = 0; % n represents the absolute number of PRM nodes (Not on a single orbit, but among all nodes)
            for i = 1:num_oribts % i is the orbit number
                obj.Stabilizers(i) = obj.Stabilizers(i).construct_reachable_FIRM_nodes();
                reachable_FIRM_nodes = obj.Stabilizers(i).reachable_FIRM_nodes;
                num_of_reachable_nodes = length(reachable_FIRM_nodes);
                disp(['Constructing FIRM nodes ',num2str(n+1:n+num_of_reachable_nodes),' out of total ',num2str(num_of_nodes),' nodes'])
                for alpha = 1:num_of_reachable_nodes  % alpha is the node number on the orbit
                    n = n+1; % n represents the absolute number of PRM nodes (Not on a single orbit, but among all PRM nodes)
                    obj.Nodes(n) = reachable_FIRM_nodes(alpha);
                    obj.Nodes(n).number = n;
                end
            end
            disp(['Time elapsed for creating ',num2str(num_of_nodes),' nodes is ',num2str(toc),' seconds'])
        end
        function obj = construct_all_FIRM_edges(obj)
            % This function constructs FIRM edges and assigns values to the "edges" property of the class.
            num_oribts = obj.PRM.num_orbits;
            tic
            n = 0; % n represents the absolute number of nodes (Not on a single orbit, but among all nodes)
            for io = 1:num_oribts % io is the orbit number
                for alpha = 1:obj.PRM.orbits(io).num_nodes % alpha is the number of node on the orbit
                    edges_from_orbit_io = find(obj.PRM.orbit_edges_list(:,1) == io);
                    for ie = edges_from_orbit_io' % IMPORTANT: in this kind of indexing, the right hand side MUST be a row vector for indexing to work.
                        tic
                        n = n+1; % absolute number of edge. Note that the numbe of FIRM edges in this case are not the same as number of PNPRM edges. Because in FIRM, we cosider node-to-orbit edges, but in PNPRM, we consider the orbit to orbit edges.
                        
                        %start_orbit_ind = obj.PRM.edges_list(ie,1); % this must be the same as "io"
                        end_orbit_ind = obj.PRM.orbit_edges_list(ie,2);
                        fprintf('Constructing edge %d, -------- starting from node %d on orbit %d to orbit %d.\n', n, alpha, io, end_orbit_ind )
                        % The following funtion concatenates a part on orbit "io" and
                        % the "PRM.orbit_edges_trajectory" to generate the node to
                        % orbit trajectories.
                        PRM_edge_traj = obj.PRM.generate_node_to_orbit_trajectory(io,alpha, end_orbit_ind);
                        % To test correcness
                        %                         for iii=1:length(PRM_edge_traj.x)
                        %                             xp=state(PRM_edge_traj.x(:,iii));
                        %                             xp.draw('RobotShape','triangle','triacolor','g','color','g');
                        %                         end
                        
                        list_of_nodes_on_starting_orbit = find(obj.PRM.corresponding_orbit == io); % The list of node indices, which lie on the starting orbit "io"
                        starting_node_ind = list_of_nodes_on_starting_orbit(alpha); % The absolute index of starting node
                        starting_FIRM_node = obj.Nodes(starting_node_ind);
                        
                        end_edge_stabilizer = obj.Stabilizers(end_orbit_ind);
                        somenumber = 1; % this number does not matter as long as it is a valid orbit number
                        possible_end_node_indices = (end_orbit_ind-1)*obj.PRM.orbits(somenumber).num_nodes+1:end_orbit_ind*obj.PRM.orbits(somenumber).num_nodes; % this line assumes the same number of nodes on every orbit
                        
                        obj.Edges(n) = FIRM_edge_class(starting_FIRM_node , possible_end_node_indices , end_edge_stabilizer , n , PRM_edge_traj );
                        obj.Edges(n) = obj.Edges(n).construct();
                        obj.Nodes(starting_node_ind).outgoing_edges = [obj.Nodes(starting_node_ind).outgoing_edges , n];
                        obj.time_of_edge_construction(n) = toc;
                    end
                end
            end
        end
        function obj = Execute(obj, initial_Hstate, start_node_ind, goal_node_ind)
            target_orbit_index = floor(goal_node_ind/3);
            target_nodes = [target_orbit_index*3,target_orbit_index*3-1, target_orbit_index*3-2];
            current_Hstate = initial_Hstate; % initialization
            current_node_ind = start_node_ind;
            while ~any(current_node_ind == target_nodes)
                next_edge_ind = obj.feedback_pi(current_node_ind); % compute the next edge (next optimal local controller) on the graph using high level feedback "pi" on the graph.
                [next_Hstate, lost, YesNo_unsuccessful, landed_node_ind] = obj.Edges(next_edge_ind).execute(current_Hstate);
                if YesNo_unsuccessful
                    disp('Ali: Execution is failed, as the robot either collided with an obstacle or ran out of time.')
                    break
                end
                if user_data_class.par.replanning == 1 % see if the replanning is allowed
                    if lost
                        next_Hstate.b.est_cov = next_Hstate.b.est_cov*2; % we increase the initial uncertainty as we assume that the estimation covariance is not realistic
                        obj.replan(next_Hstate, goal_node_ind); % Note that this function should not output "obj". The reason is explained inside the function.
                        return; % after replanning, we do not continue the loop anymore, becuase the whole plan is changed.
                    end
                end
                current_Hstate = next_Hstate;
                current_node_ind = landed_node_ind;
            end
        end
        function obj = Execute_with_replanning(obj,start_node_ind,goal_node_ind)
            error('not updated yet')
            n_samples = 2; % you will see the reason for this "two" in the next lines' comments.
            initial_robot_ensemble_temp = obj.Nodes(start_node_ind).sample(n_samples); % in this function, we do not propagate the ensemble. We only pick a single hstate from the ensemble and propagate it.
            for q = 2:length(initial_robot_ensemble_temp.Hparticles) % here we can just pick one of particle. or run for all particles. Just remember the first one is the no-noise particle, so we may start from the second particle.
                initial_hstate = initial_robot_ensemble_temp.Hparticles(q);
                Execute_with_replanning_single_robot(obj,initial_hstate,start_node_ind,goal_node_ind);
            end
        end
    end
    
    methods (Access = private)
        function Execute_with_replanning_single_robot(obj,initial_hstate,start_node_ind,goal_node_ind)
            % Note that we must NOT output the "obj" in
            % this function, because we do not want the newly added node
            % gets added to the main graph. We want the other robot (the
            % next runs), use the same original graph. Thus, we want to treat the new nodes as the
            % temporary nodes.
            current_hstate = initial_hstate;
            current_node_ind = start_node_ind;
            list_of_edges = obj.PRM.edges_list;
            while current_node_ind ~= goal_node_ind
                next_edge_ind = obj.feedback_pi(current_node_ind); % compute the next edge number (next local controller) on the graph using high level feedback "pi" on the graph.
                current_GHb = obj.Nodes(current_node_ind).center_GHb;
                [next_hstate, lost, failed] = obj.Edges(next_edge_ind).execute_single_robot_with_replanning(current_hstate, current_GHb); % current_GHb is really not needed here. It is only for plotting purposes.
                if failed
                    break
                end
                if lost
                    new_node_ind = obj.PRM.num_nodes + 1; % this is the number of first newly added node (which coincides the existing estimation right at this point).
                    obj.PRM = obj.PRM.add_node(next_hstate.b.est_mean); % add a node to PRM
                    obj = obj.add_a_node_and_its_sequel(next_hstate.b); % add a node to FIRM
                    if user_data_class.par.goBack_to_nearest_node
                        nearest_node_ind = obj.PRM.compute_nearest_node_ind(new_node_ind);
                        obj.feedback_pi(new_node_ind) = nearest_node_ind;
                    else
                        % in FIRM, we need to update the "feedback pi" too. Note that we are doing this in a naive way by updating whole values. The computationally less expensive way is only to compute the newly added node. However, the benefit of the current method is if we want to add the node to the graph (i.e. the new nodes which comes later can get connected to it.) this is correct to update the whole values.
                        obj = obj.DP_compute_cost_to_go_values(goal_node_ind);
                    end
                    obj.Execute_with_replanning_single_robot(next_hstate,new_node_ind,goal_node_ind);
                    return
                end
                current_hstate = next_hstate;
                current_node_ind = next_node_ind;
            end
        end
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
            obj.num_edges = size(obj.PRM.edges_list,1);disp('i THINK NOT CORRECT IN PERIODIC CASE ?????')
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
    
end
