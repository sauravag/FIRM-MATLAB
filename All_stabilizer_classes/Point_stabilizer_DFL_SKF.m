%% Class Definition
classdef Point_stabilizer_DFL_SKF < Stabilizer_interface
    %==============================  FIRM node based on Nonlinear Unicycle Controller (Oriolo's paper) and SKF (Stationary KF) =========================================
    % This stabilizer is
    % based on Nonlinear Unicycle Controller (Oriolo's paper) and SKF 
    properties (Access = private)
        controller; % This is the nonlinear controller (based on oriolo's paper) designed for the unicycle.
        PRM_node; % Underlying PRM node
        PRM_node_number; % The FIRM node number in the entire FIRM graph
        plot_handle;
    end
    
    methods
        function obj = Point_stabilizer_DFL_SKF(PRM_node_inp)
            if nargin>0
                obj.PRM_node = PRM_node_inp;
                obj.controller = DFL_SKF_class(PRM_node_inp.val); % Note that the node controller is an object of "DFL_SKF_class" NOT simple "DFL_SKF_class" itself.
                obj.par = user_data_class.par.stabilizer_parameters;
            end
        end
        function obj = construct_reachable_FIRM_nodes(obj)
            stationary_Pest = obj.controller.Stationary_Pest;
            center_bel = belief(obj.PRM_node , stationary_Pest);
            FIRM_node = FIRM_node_class( center_bel );
            %FIRM_node.center_GHb = stGHb; % GHb is not meaningful for
            %DFL_SKF
            obj.reachable_FIRM_nodes = FIRM_node;
        end
        function [seq_of_PHb, seq_of_GHb, target_reaching_probabilities] = construct_seq_of_Hb(obj,initial_PHb, initial_GHb)
            if ~user_data_class.par.No_plot
                % Zoom in to the node area in the figure
                old_zoom = obj.PRM_node.zoom_in(5); % the input argument is the zoom ratio (wrt the axis size)
                xlabel(['Point stabilizer ',num2str(obj.stabilizer_number),' is working ...']);
            end
            disp(['Point stabilizer ',num2str(obj.stabilizer_number),' is working ...'])
            % initializing the particle Hbelief sequence
            seq_of_PHb = Hbelief_p.empty; % Without this line we get the "object conversion error to double". There should be a better way of overcoming this problem, but I do not know at this time.
            seq_of_PHb(1) = initial_PHb;
            % initializing the Gaussian Hbelief sequence
            seq_of_GHb = Hbelief_G.empty; % Without this line we get the "object conversion error to double". There should be a better way of overcoming this problem, but I do not know at this time.
            seq_of_GHb(1) = initial_GHb;
            k = 1; % k denotes the time step
            disp('AliFW: Following memory preallocatoins are too conservative. Fix it please.')
            seq_of_PHb(1,2*obj.par.max_stopping_time) = Hbelief_p; % memory preallocation for PHb. We allocate the twice of the "maximum particle stopping time", because, usually the convergence time is less than "maximum particle stopping time". Note that we will cut this sequence after the exact length of sequence is found.
            seq_of_GHb(1,2*obj.par.max_stopping_time) = Hbelief_G; % memory preallocation for PHb. We allocate the twice of the "maximum particle stopping time", because, usually the convergence time is less than "maximum particle stopping time". Note that we will cut this sequence after the exact length of sequence is found.
            
            % following loop propagates both PHb and GHb until the stopping
            % condition is satisfied.
            all_prtcls_stopped = 0; % initial value
            convergence_happend = 1; % initial value
            convergence_time = 0; % we do not output this variable anymore, because the length of following sequence is the same as this variable.
                        
            landed_nodes = zeros(1,user_data_class.par.par_n); % q-th entry of "landed_node" tells that which node the q-th particle has landed in. % In this class all of its entries will be the same, since we only have a single target node.
            global Previous_control;
            Previous_control = zeros(MotionModel_class.ctDim , 1);  % Since we are using a "Dynamic feedback linearization based" controller, at each step
            % we need to have the prevous control signal value. So, every
            % time we are starting a new edge, we have to initialize it be
            % zero here.
            while ~all_prtcls_stopped
                disp(['Step ',num2str(k),' of FIRM stabilizer ',num2str(obj.stabilizer_number),'. Hb has convereged at ',num2str(convergence_time),' and max stopping time after that is ',num2str(obj.par.max_stopping_time),' .']);
                % in following we compute the sequence of Particle-Hbelief
                % induced by the node controller (stationary-LQG)
                seq_of_PHb(k+1) = obj.controller.propagate_Hb_particle(seq_of_PHb(k));
                if ~user_data_class.par.No_plot
                    if obj.par.draw_cov_centered_on_nominal == 1 % in this case, we do NOT draw estimation covariance centered at estimation mean. BUT we draw estimation covariance centered at nominal state locations, to illustrate the covariance convergence.
                        nominal_x = obj.PRM_node;
                        seq_of_PHb(k+1) = seq_of_PHb(k+1).draw_CovOnNominal(nominal_x , 'robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r');
                        seq_of_PHb(k) = seq_of_PHb(k).delete_plot();
                    else  % This is the normal case, where we draw the estimation covariance centered at estimation mean.
                        seq_of_PHb(k+1) = seq_of_PHb(k+1).draw('robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r');
                        seq_of_PHb(k) = seq_of_PHb(k).delete_plot();
                    end
                    drawnow
                end
                % after convergence to the stationary_GHb happens, we start
                % to check if FIRM stopping condition is satisfied or not.
                if convergence_happend
                    % Following function check if the particles enter the
                    % stopping region or not. Moreover, it set the
                    % "stopping_times" property of the "Hbelief_p" to its
                    % corresponding value. Therefore, we provide the time
                    % k+1 as well, among input arguments.
                    [seq_of_PHb(k+1),landed_nodes] = obj.stopping_condition_check_for_PHb(seq_of_PHb(k+1),k+1,convergence_time,landed_nodes);  % Note that both input and output are in time step k+1. This function updates the "stopped_particles" and "unsuccessful_particles" properties of the PHb at time step k+1.
                    all_prtcls_stopped = all(seq_of_PHb(k+1).stopped_particles | seq_of_PHb(k+1).collided_particles | seq_of_PHb(k+1).unsuccessful_particles); % indicates if all particles are stopped or collided.
                end
                if (user_data_class.par.sim.video == 1 && ~user_data_class.par.No_plot)
                    global vidObj; %#ok<TLEV>
                    currFrame = getframe(gcf);
                    writeVideo(vidObj,currFrame);
                end
                
                k = k+1; % progress the time
            end
            seq_of_PHb = seq_of_PHb(1:k); % Here we cut the extra part of "seq_of_PHb".
            target_reaching_probabilities = zeros(1,length(obj.reachable_FIRM_nodes)); % it has to be initialized to "scalar zero" in this class, since we only have a single possible target node.
            for i = 1:length(obj.reachable_FIRM_nodes)
                target_reaching_probabilities(i) = length(find(landed_nodes==obj.stabilizer_number))/user_data_class.par.par_n;
            end
            
            % zoom out
            if ~user_data_class.par.No_plot
                axis(old_zoom);
            end
        end
        function [next_Hstate, lost, YesNo_unsuccessful, landed_node_ind] = execute(obj, current_Hstate, convergence_time)
            % This function stabilizes the belief in a runtime and
            % terminates if replanning is needed (if the user has turned "on" the replanning flag).
            
            % Zoom in to the node area in the figure
            old_zoom = obj.PRM_node.zoom_in(5); % the input argument is the zoom ratio (wrt the axis size)
            xlabel(['Point stabilizer ', num2str(obj.stabilizer_number),' is working ...']);
            show_just_once = 1; % this is an auxiliary variable that cause the figure lable is just updated once.
            k = 1;
            stop_flag = 0; % initialization
            lost = 0; % initialization % only needed if the "replanning flag" is turned on by user.
            global Previous_control;
            Previous_control = zeros(MotionModel_class.ctDim , 1);  % Since we are using a "Dynamic feedback linearization based" controller, at each step
            % we need to have the prevous control signal value. So, every
            % time we are starting a new edge, we have to initialize it be
            % zero here.
            
            while ~stop_flag
                disp(['Step ',num2str(k),' of point stabilizer ',num2str(obj.stabilizer_number),'. convergence time is ',num2str(convergence_time),'.']);
                next_Hstate = obj.controller.propagate_Hstate(current_Hstate);
                if obj.par.draw_cov_centered_on_nominal == 1 % in this case, we do NOT draw estimation covariance centered at estimation mean. BUT we draw estimation covariance centered at nominal state locations, to illustrate the covariance convergence.
                    nominal_x = obj.PRM_node;
                    next_Hstate = next_Hstate.draw_CovOnNominal(nominal_x);
                    current_Hstate = current_Hstate.delete_plot(); %#ok<NASGU>
                else  % This is the normal case, where we draw the estimation covariance centered at estimation mean.
                    next_Hstate = next_Hstate.draw();
                    current_Hstate = current_Hstate.delete_plot(); %#ok<NASGU>
                end
                if user_data_class.par.replanning == 1
                    % Here, we check if we lie in the valid linearization
                    % region of node controller or not
            
                    signed_elem_wise_difference = next_Hstate.b.est_mean.signed_element_wise_dist(obj.PRM_node);

                    is_in_end_node_lnr_domain = all(abs(signed_elem_wise_difference) < user_data_class.par.valid_linearization_domain); % never forget the "absolute value operator" in computing distances.
                    lost = ~is_in_end_node_lnr_domain;
                else
                    lost = 0; % By this, we indeed disable the replanning
                end
                
                bel = next_Hstate.b;
                candidate_FIRM_node = obj.reachable_FIRM_nodes; % In this "class", we only have a single reachable node, so we put it in "candidate_FIRM_node".
                YesNo_reached = candidate_FIRM_node.is_reached(bel); % Here, we check if the belief enters the stopping region or not.
                YesNo_timeout = (k+1-convergence_time > obj.par.max_stopping_time);
                YesNo_collision = next_Hstate.Xg.is_constraint_violated();
                stop_flag = YesNo_reached || YesNo_timeout || YesNo_collision || lost;
                if show_just_once == 1 % we do not want to update the xlabel at each step.
                    xlabel('Hbelief has converged. Particles  are trying to reach stopping region...')
                    disp('Hbelief has converged. Particles are trying to reach stopping region...')
                    show_just_once = 0;
                end
                drawnow
                % making a video of runtime
                if user_data_class.par.sim.video == 1
                    global vidObj; %#ok<TLEV>
                    currFrame = getframe(gcf);
                    writeVideo(vidObj,currFrame);
                end
                % update the ensemble and GHb
                current_Hstate = next_Hstate;
                k = k+1;
            end
            YesNo_unsuccessful =  YesNo_timeout || YesNo_collision;
            if YesNo_unsuccessful == 1
                landed_node_ind = 'No landed node! the execution was unsuccessful.';
            else
                landed_node_ind = obj.stabilizer_number; % This line should be updated, since the "stabilizer_number" should not be used for any reason other than displaying purposes. It is not really part of  the class.
            end
            
            axis(old_zoom);
        end
    end
    
    methods (Access = private)
        function [updated_PHb , landed_node_ind] = stopping_condition_check_for_PHb(obj,PHb,time,convergence_time,landed_node_ind)
            if ~exist('landed_node_ind','var')
                landed_node_ind = [];
            end
            % This function updates the "stopped_particles" and "unsuccessful_particles" properties of the PHb at time step k+1.
            updated_PHb = PHb; % we preserve all the information in PHb and only update the its "stopped_particles" property.
            alive_particles = find(~PHb.stopped_particles & ~PHb.collided_particles); % alive particles are those that have not collided or stopped yet.
            % if the time is greater than the maximum allowed time, alive
            % particles are considered as the unsuccessful particles.
            if time - convergence_time > obj.par.max_stopping_time
                updated_PHb.unsuccessful_particles(alive_particles) = 1;
                updated_PHb.stopping_times(alive_particles) = time;
            else % if the time is OK, for alive particles, we check if the
                % stopping condition is satisfied or not
                for q = alive_particles' % note that "alive_particles" vector has to be a row vector for this line to work properly. NOT a column vector.
                    bel = PHb.Hparticles(q).b;
                    candidate_FIRM_node = obj.reachable_FIRM_nodes; % In this "class", we only have a single reachable node, so we put it in "candidate_FIRM_node".
                    if candidate_FIRM_node.is_reached(bel)  % Here, we check if the belief enters the stopping region or not.
                        updated_PHb.stopped_particles(q) = 1;
                        updated_PHb.stopping_times(q) = time;
                        landed_node_ind(q) = obj.stabilizer_number;  % in this class the "node_number" is the same as the "stabilizer number", and hence this line. % q-th entry of "landed_node" tells that which node the q-th particle has landed in. % However, I think this is not a right way of treating "landed_nodes"; I think the node number has to be returned locally. I mean this class has to use "stabilizer_number" only for illustration purposes and indeed should not know which stabilizer or node it is. That is the job of FIRM_graph_class to assign the locally retrurned "landed_nodes" to the globally indexed "landed_nodes".
                    end
                end
                
            end
        end
    end
end
