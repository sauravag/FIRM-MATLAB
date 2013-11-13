classdef FIRM_edge_class
    % FIRM_edge_class encapsulates the properties and methods associated with defining FIRM edges.
    %test
    properties
        start_node; % starting node of the edge. Note that we can only have one "start_node".
        possible_end_node_indices;  % The indices of possible target nodes of the edge. Note that we can have multiple "end_nodes" since we deal with stochastic edges; that is why the property name is NOT "end_node".
        target_node_stabilizer; % The object, which stabilize the belief to one of the "possible_end_nodes".
        number; % absolute edge number (index).
        kf; % Nominal length of the edge. This length comes through designing the nominal control sequence u.
        filtering_cost; % The filtering cost associated with the FIRM edge.
        FIRM_stopping_time; % The stopping time of the FIRM edge (stochastic length of the edge).
        FIRM_cost; % Total cost associated with the FIRM edge.
        failure_probability; % Failure probability ( the probability of NOT reaching a target node.)
        reaching_probabilities; % A vector with the same size as "end_nodes" that describes the probability of reaching to the target nodes.
    end
    
    properties (Access = private)
        nominal_trajectory;
        PHb_seq;
        GHb_seq;
        PHb_seq_induced_by_end_stabilizer;
        GHb_seq_induced_by_end_stabilizer;
        edge_controller;
        HBelief_convergence_time; % The convergence time of Hyper belief.
    end
    
    methods
        function obj = FIRM_edge_class(start_node_inp  ,  possible_end_node_indices_inp  ,  target_node_stabilizer_inp  ,  edge_number_inp  ,  nominal_PRM_edge_traj)
            if nargin>0
                obj.start_node = start_node_inp;
                obj.target_node_stabilizer = target_node_stabilizer_inp;
                obj.number = edge_number_inp;
                obj.nominal_trajectory = nominal_PRM_edge_traj;
                obj.possible_end_node_indices =possible_end_node_indices_inp;
                % obj.edge_controller = target_node_stabilizer_inp.controller; % If this line is uncommented, it means we assume the edge controller is the same as the stationary_LQG designed for end_node.
                obj.edge_controller = LQG_class('LKF',obj.nominal_trajectory);
                obj.kf = obj.edge_controller.kf;
            end
        end
        function obj = construct(obj)
            start_node_ind = obj.start_node.number; % for display only
            % computing initial GHb and PHb of the edge
            % Xg_mean = obj.start_node.node_center.est_mean;
            Xest_MeanOfMean = obj.start_node.center_b.est_mean;
            Xg_mean = Xest_MeanOfMean;
            Pest = obj.start_node.center_b.est_cov;
            stdim = state.dim;
            P_of_joint = [Pest , zeros(stdim,stdim)  ;  zeros(stdim,stdim) , eps*eye(stdim) ];
            
            if max(max(abs(Pest - Pest')))<1e-8,  Pest = (Pest+Pest')/2; else error('Pest is too unsymmetric'); end
            if max(max(abs(P_of_joint - P_of_joint')))<1e-8,  P_of_joint = (P_of_joint+P_of_joint')/2; else error('P_of_joint is too unsymmetric'); end
            
            initial_GHb = Hbelief_G(Xg_mean, Xest_MeanOfMean, Pest, P_of_joint);
            
            initial_PHb = initial_GHb.HbeliefG_to_HbeliefP(user_data_class.par.par_n);
            %              initial_PHb = obj.start_node.sample_stationary_GHb();
            % draw initial PHb
            if ~user_data_class.par.No_plot
                initial_PHb = initial_PHb.draw();
                xlabel(['Edge controller of edge ',num2str(obj.number),' starting from node ',num2str(start_node_ind), ' is working']);
            end
            % initializing the first element in the sequence
            obj.PHb_seq = Hbelief_p.empty;
            obj.PHb_seq(1) = initial_PHb;
            obj.GHb_seq = Hbelief_G.empty;
            obj.GHb_seq(1) = initial_GHb;
            
            % in the following loop we propagate the PHb (and maybe GHb)
            % along the edge, using the edge_controller.
            for k = 1 : obj.kf
                disp(['Step ',num2str(k),' of edge controller of edge ',num2str(obj.number),' starting from node ',num2str(start_node_ind) ]);
                % propagation of PHb
                obj.PHb_seq(k+1) = obj.edge_controller.propagate_Hb_particle(obj.PHb_seq(k),k);
                if ~user_data_class.par.No_plot
                    obj.PHb_seq(k+1) = obj.PHb_seq(k+1).draw();
                    obj.PHb_seq(k) = obj.PHb_seq(k).delete_plot();
                    drawnow
                end
                % propagation of GHb (only meaningful for when the filter
                % is LKF)
                
                obj.GHb_seq(k+1) = obj.edge_controller.propagate_Hb_Gaussian(obj.GHb_seq(k),k);
                if ~user_data_class.par.No_plot
                    %                     obj.GHb_seq(k+1) = obj.GHb_seq(k+1).draw();
                    %                     obj.GHb_seq(k) = obj.GHb_seq(k).delete_plot();
                    %                     drawnow
                end
                
                if (user_data_class.par.sim.video == 1 && ~user_data_class.par.No_plot)
                    global vidObj; %#ok<TLEV>
                    currFrame = getframe(gcf);
                    writeVideo(vidObj,currFrame);
                end
                
                % collision check
                obj.PHb_seq(k+1) = obj.PHb_seq(k+1).collision_check(); % this function updates the "collided_particles" property of the "PHb_seq(k+1)"
            end
            % followin function of the "end_node" returns the
            % "initial_PHb_seq" and "initial_GHb_seq" induced by the
            % "node_controller".
            [obj.PHb_seq_induced_by_end_stabilizer, obj.GHb_seq_induced_by_end_stabilizer , obj.reaching_probabilities] = obj.target_node_stabilizer.construct_seq_of_Hb(obj.PHb_seq(k+1), obj.GHb_seq(k+1));
            % compute the filtering_cost of edge
            obj.filtering_cost = obj.Compute_filtering_cost();
            % compute the stopping times of HBRM and FIRM
            [obj.HBelief_convergence_time, obj.FIRM_stopping_time] = obj.Compute_stopping_times();
            % compure the failure probabilitiy
            obj.failure_probability = obj.Compute_failure_probability();
            % compute the total cost of edge
            obj.FIRM_cost = obj.Compute_totoal_FIRM_cost();
            if user_data_class.par.No_history
                obj.GHb_seq = [];
                obj.PHb_seq = [];
                obj.PHb_seq_induced_by_end_stabilizer = [];
                obj.GHb_seq_induced_by_end_stabilizer = [];
                disp('Here, we have to delete the data inside the "edge_controller" object to free the memory.')
            end
        end
        function [next_Hstate, lost, YesNo_unsuccessful, landed_node_ind] = execute(obj,init_hstate)
            % This function executes the feedback plan for a single robot (hstate). Also, in case the "replanning flag" is turned on by the user, then if it
            % deviates from the nominal path significantly, the function
            % returns "lost = 1".
            
            current_Hstate = init_hstate;
            xlabel(['Edge controller of edge ',num2str(obj.number),' starting from node ',num2str(obj.start_node.number),' is working']);
            % edge part
            draw_at_every_n_steps = user_data_class.par.draw_at_every_n_steps;
            for k = 1 : obj.kf
                % propagation of a single robot (or an Hstate)
                if user_data_class.par.replanning == 1
                    [next_Hstate, in_lnr_reg] = obj.edge_controller.propagate_Hstate(current_Hstate,k);
                    lost = ~in_lnr_reg;
                else
                    next_Hstate = obj.edge_controller.propagate_Hstate(current_Hstate,k);
                    lost = 0;
                end
                next_Hstate.Xg = obj.apply_disturbance(next_Hstate.Xg); % this function applies the disturbance on the "Xg", if mouse is clicked on the axes.
                if mod(k,draw_at_every_n_steps)==0
                    next_Hstate = next_Hstate.draw();
                end
                current_Hstate = current_Hstate.delete_plot();
                drawnow
                if mod(k,draw_at_every_n_steps)==0
                    % making video of run-time simulation
                    if user_data_class.par.sim.video == 1
                        global vidObj; %#ok<TLEV>
                        currFrame = getframe(gcf);
                        writeVideo(vidObj,currFrame);
                    end
                end
                % collision check
                YesNo_unsuccessful = current_Hstate.Xg.is_constraint_violated();
                
                if YesNo_unsuccessful || lost
                    landed_node_ind = 'No landed node! the execution was unsuccessful.';
                    return
                end
                % updating the Hstate
                current_Hstate = next_Hstate;
            end
            convergence_time = 0; % This is zero because, we right now do not consider GHb convergence as a pre-condition for FIRM node reaching. Otherwise, the following line has to be uncommented.
            % convergence_time = obj.HBelief_convergence_time - obj.kf;
            [next_Hstate, lost, YesNo_unsuccessful, landed_node_ind] = ...
                obj.target_node_stabilizer.execute(current_Hstate,convergence_time);
        end
    end
    
    methods (Access = private)
        function cost = Compute_filtering_cost(obj)
            not_normalized_filtering_cost = 0;
            alive_particles = find(~obj.PHb_seq(end).collided_particles)'; % note that here "alive_particles" has to be a row vector
            for q = alive_particles
                for k = 1 : obj.kf
                    % note that to compute the filtering cost, we only
                    % consider the edge part not node part.
                    not_normalized_filtering_cost = not_normalized_filtering_cost + trace(obj.PHb_seq(k).Hparticles(q).b.est_cov);
                end
            end
            if isempty(alive_particles)
                cost = 1000; % if no particle can reach the end node, filtering cost is set to 1000
            else
                cost = not_normalized_filtering_cost/(obj.kf*length(alive_particles));
            end
        end
        function [HBRM_time,FIRM_time] = Compute_stopping_times(obj)
            HBRM_time = obj.kf + length(obj.GHb_seq_induced_by_end_stabilizer);
            if strcmpi(user_data_class.par.RoadMap,'FIRM')
                ind = find(obj.PHb_seq_induced_by_end_stabilizer(end).stopping_times > 0); % since the "stopping times" are initialized by (-1), the stopping time of collided particles are still (-1) at this point. Thus, this line exclude those particles in computing the mean stopping time.
                if isempty(ind)
                    FIRM_time = 10^6; % If no particles can reach the end node in the maximum allowed time, the FIRM time is set to a large number.
                else
                    FIRM_time = obj.kf + mean(obj.PHb_seq_induced_by_end_stabilizer(end).stopping_times(ind));
                end
            else
                FIRM_time = 'we are in HBRM!'; % in HBRM, we do not need to compute the FIRM stopping time.
            end
        end
        function failure_prob = Compute_failure_probability(obj)
            % compute the failure probability
            if strcmpi(user_data_class.par.RoadMap,'FIRM')
                % Note that in FIRM failed particles are collided ones plus the
                % unsuccessful ones.
                % VERY IMPORTANT: To compute the collision probability we only
                % use the edge part, i.e. "obj.PHb_seq(obj.kf)", but for the
                % unsuccessful particles we use the node part, i.e.
                % "PHb_seq_induced_by_end_stabilizer(end)".
                num_failed_prtcls = length(find(obj.PHb_seq(obj.kf).collided_particles | obj.PHb_seq_induced_by_end_stabilizer(end).unsuccessful_particles)); % number of collided or unsuccessful particles along the edge
            elseif strcmpi(user_data_class.par.RoadMap,'HBRM')
                num_failed_prtcls = length(find(obj.PHb_seq(obj.kf).collided_particles)); % number of collided particles along the edge
            end
            failure_prob  = num_failed_prtcls/obj.PHb_seq(obj.kf).num_p;  % OR  failure_prob  = num_failed_prtcls/user_data_class.par.par_n;
        end
        function total_cost = Compute_totoal_FIRM_cost(obj)
            disp('AliFW: in computing total cost, now, we are NOTTT using the stopping time along with the filtering cost.')
            total_cost = obj.filtering_cost;% + obj.FIRM_stopping_time;
        end
        function disturbed_Xg = apply_disturbance(obj,Xg) %#ok<MANU>
            disturbed_Xg = Xg;
            if user_data_class.par.disturbance_allowed
                global last_click_position; %#ok<TLEV>
                if isempty(last_click_position)
                    last_click_position = get(gca,'CurrentPoint');
                end
                current_click_position = get(gca,'CurrentPoint');
                if any(current_click_position(1,1:2) ~= last_click_position(1,1:2))
                    limits = axis;
                    within_axis = (current_click_position(1,1)>limits(1) && current_click_position(1,1)<limits(2) && current_click_position(1,2)>limits(3) && current_click_position(1,2)<limits(4));
                    if within_axis
                        disturbed_Xg.val(1:state.dim-1) = [current_click_position(1,1);current_click_position(1,2)];
                    end
                end
                last_click_position = current_click_position;
            end
            persistent delete_ali
                if isempty(delete_ali), delete_ali = 1; else, delete_ali = delete_ali + 1; 
                end
                if delete_ali<15
%                 disturbed_Xg.val = [disturbed_Xg.val(1)+rand*1;disturbed_Xg.val(2)-rand*1;delete_ali*2*pi/180];  % I add this to prepare an specific disturbance for the needle steering procedure. You have to remove it.
                end
        end
    end
    
end