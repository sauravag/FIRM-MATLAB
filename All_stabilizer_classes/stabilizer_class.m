classdef stabilizer_class < Stabilizer_interface
    % Orbit_stabilizer_PLQG_class encapsulates the belief orbit stabilizer
    % concept in Periodic-node FIRM, which is implemented using Periodic
    % LQG controllers.
    
    properties (Access = private)
        controller; % the controller utilizerd to stabilize the belief
        PRM_orbit; % underlying PRM orbit
        PRM_nodes_on_orbit; % underlying PRM nodes on the orbit
        PRM_orbit_number; % PRM orbit number.
        absolute_node_numbers; % The absolute number of PRM nodes (ie among all PRM nodes) on the orbit.
        belief_orbit; % The orbit of "belief mean"s.
        plot_handle; % plot handle associated with plots drawn by stabilizer class
    end
    
    methods
        function obj = stabilizer_class(PRM_nodes_on_orbit_inp  ,  PRM_orbit_inp)  % Two first inputs are only for consistency with the other FIRM_node_classes. This issue should be resolved.
            if nargin > 0
                obj.PRM_nodes_on_orbit = PRM_nodes_on_orbit_inp;
                obj.PRM_orbit = PRM_orbit_inp;
                %                 obj.PRM_orbit_number = stabilizer_number_inp;
                obj.controller = LQG_periodic_class(PRM_orbit_inp); % Note that the node controller is an object of "LQG_periodic_class" NOT simple "LQG".
                obj.par = user_data_class.par.stabilizer_parameters;
            end
        end
        function obj = construct_reachable_FIRM_nodes(obj)
            % periodic_GHb = obj.controller.Periodic_Gaussian_Hb;  % This function has not been written for the periodic case yet.
            % obj.node = stGHb; % No Hblief is implemented in PLQG yet
            obj.belief_orbit =  obj.controller.periodic_belief;
            node_time_stages_on_orbit = obj.PRM_orbit.node_time_stages;
            for i = 1:length(node_time_stages_on_orbit)
                time_stage = node_time_stages_on_orbit(i);
                belief_at_that_time_stage = obj.belief_orbit(time_stage);
                obj.reachable_FIRM_nodes(i) = FIRM_node_class( belief_at_that_time_stage );
            end
        end
        function [seq_of_PHb, seq_of_GHb, target_reaching_probabilities] = construct_seq_of_Hb(obj,initial_PHb, initial_GHb) %#ok<INUSD>
            if ~user_data_class.par.No_plot
                % Zoom in to the node area in the figure
                old_zoom = obj.zoom_in();
                xlabel(['Orbit stabilizer',num2str(obj.stabilizer_number),' is working ...']);
            end
            disp(['Orbit stabilizer',num2str(obj.stabilizer_number),' is working ...'])
            % initializing the particle Hbelief sequence
            seq_of_PHb = Hbelief_p.empty; % Without this line we get the "object conversion error to double". There should be a better way of overcoming this problem, but I do not know at this time.
            seq_of_PHb(1) = initial_PHb;
            % initializing the Gaussian Hbelief sequence
            % The GHb has not been implemented yet in periodic case.
            % seq_of_GHb = Hbelief_G.empty; % Without this line we get the "object conversion error to double". There should be a better way of overcoming this problem, but I do not know at this time.
            % seq_of_GHb(1) = initial_GHb;
            
            k = 1; % k denotes the time step
            disp('AliFW: Following memory preallocatoins are too conservative. Fix it please.')
            seq_of_PHb(1,2*obj.par.max_stopping_time) = Hbelief_p; % memory preallocation for PHb. We allocate the twice of the "maximum particle stopping time", because, usually the convergence time is less than "maximum particle stopping time". Note that we will cut this sequence after the exact length of sequence is found.
            % seq_of_GHb(1,2*user_data_class.par.max_stopping_time) = Hbelief_G; % memory preallocation for PHb. We allocate the twice of the "maximum particle stopping time", because, usually the convergence time is less than "maximum particle stopping time". Note that we will cut this sequence after the exact length of sequence is found.
            
            % following loop propagates both PHb and GHb until the stopping
            % condition is satisfied.
            all_prtcls_stopped = 0; % initial value
            convergence_happend = 1; % initial value % ************  In Periodic case, we have not implemented the GHb yet. So, by making the "convergence_happend", we are DISABLING the convergence check based on the GHb.
            convergence_time = 0; % '??'; % initilization. For display only.  % ************  In Periodic case, since we have DISABLED the convergence check based on the GHb, we set the "convergence_time = 0" so that it makes sense.
            landed_nodes = zeros(1,user_data_class.par.par_n); % q-th entry of "landed_node" tells that which node the q-th particle has landed in.
            while ~all_prtcls_stopped
                disp(['Step ',num2str(k),' of FIRM stabilizer ',num2str(obj.stabilizer_number),'. Hb has convereged at ',num2str(convergence_time),' and max stopping time after that is ',num2str(obj.par.max_stopping_time),' .']);
                % in following we compute the sequence of Particle-Hbelief
                % induced by the node controller (periodic-LQG)
                seq_of_PHb(k+1) = obj.controller.propagate_Hb_particle(seq_of_PHb(k), k);  % Note that PLQG is a time-varying controller; So, we have to provide time "k" as an input too.
                if ~user_data_class.par.No_plot
                    if obj.par.draw_cov_centered_on_nominal == 1 % in this case, we do NOT draw estimation covariance centered at estimation mean. BUT we draw estimation covariance centered at nominal state locations, to illustrate the covariance convergence.
                        T = obj.PRM_orbit.period;
                        cyclic_kPlus1 = mod(k+1,T)+(T*(mod(k+1,T)==0));
                        nominal_x = state(obj.controller.lnr_pts_periodic(cyclic_kPlus1).x);
                        seq_of_PHb(k+1) = seq_of_PHb(k+1).draw_CovOnNominal(nominal_x , 'robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r');
                        seq_of_PHb(k) = seq_of_PHb(k).delete_plot();
                    else  % This is the normal case, where we draw the estimation covariance centered at estimation mean.
                        seq_of_PHb(k+1) = seq_of_PHb(k+1).draw('robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r');
                        seq_of_PHb(k) = seq_of_PHb(k).delete_plot();
                    end
                    drawnow
                end
                % in following we compute the sequence of Gaussian-Hbelief
                % induced by the node controller (periodic-LQG)
                if ~convergence_happend % we do NOT propagate the GHb after it converges to the stationary GHb
                    seq_of_GHb(k+1) = obj.controller.propagate_Hb_Gaussian(seq_of_GHb(k));
                    if ~user_data_class.par.No_plot
                        seq_of_GHb(k+1) = seq_of_GHb(k+1).draw();
                        seq_of_GHb(k) = seq_of_GHb(k).delete_plot();
                        drawnow
                    end
                    % check if the GHb is converged to the stationary GHb of
                    % the node or not.
                    YesNo = obj.Is_GHb_converged(seq_of_GHb(k+1));
                    if YesNo == 1
                        convergence_happend = 1;
                        convergence_time = k+1; % we do not output this variable anymore, because the length of following sequence is the same as this variable.
                        seq_of_GHb = seq_of_GHb(1:k+1); % Here we cut the extra part of the "seq_of_GHb".
                        xlabel('Hbelief has converged. Particles trying to reach stopping region...')
                    end
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
                k = k+1; % progress the time
            end
            seq_of_PHb = seq_of_PHb(1:k); % Here we cut the extra part of "seq_of_PHb".
            seq_of_GHb = 'GHb has not been implemented in Periodic case yet.';
            target_reaching_probabilities = zeros(1,length(obj.reachable_FIRM_nodes));
            for i = 1:length(obj.reachable_FIRM_nodes)
                target_reaching_probabilities(i) = length(find(landed_nodes==i))/user_data_class.par.par_n;
            end
            % zoom out
            if ~user_data_class.par.No_plot
                obj.zoom_out(old_zoom);
            end
        end
        function [next_Hstate, lost, YesNo_unsuccessful, absolute_landed_node_ind] = execute(obj,current_Hstate, convergence_time)
            % This function stabilizes the belief in a runtime and
            % terminates if replanning is needed (if the user has turned "on" the replanning flag).
            
            % Zoom in to the node area in the figure
            center_state = state( [obj.PRM_orbit.center ; 0 ]); % This is a very poor code line, as it only works for 3D state. I need to fix it ASAP.
            old_zoom = center_state.zoom_in(5); % the input argument is the zoom ratio (wrt the axis size)
            xlabel(['Orbit stabilizer number ',num2str(obj.stabilizer_number),' is working ...']);
            
            show_just_once = 1;
            k = 1;
            stop_flag = 0; % initialization
            lost = 0; % initialization % only needed if the "replanning flag" is turned on by user.
            while ~stop_flag
                                disp(['Step ',num2str(k),' of orbit stabilizer number ',num2str(obj.stabilizer_number),'. convergence time is ',num2str(convergence_time),'.']);
                next_Hstate = obj.controller.propagate_Hstate(current_Hstate,k); % note that periodic LQG is a time-varying controller. So, it needs "k" as an input.
                if obj.par.draw_cov_centered_on_nominal == 1 % in this case, we do NOT draw estimation covariance centered at estimation mean. BUT we draw estimation covariance centered at nominal state locations, to illustrate the covariance convergence.
                    T = obj.PRM_orbit.period;
                    cyclic_kPlus1 = mod(k+1,T)+(T*(mod(k+1,T)==0));
                    nominal_x = state(obj.controller.lnr_pts_periodic(cyclic_kPlus1).x);
                    next_Hstate = next_Hstate.draw_CovOnNominal(nominal_x,'robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r');
                    current_Hstate = current_Hstate.delete_plot(); %#ok<NASGU>
                else  % This is the normal case, where we draw the estimation covariance centered at estimation mean.
                    next_Hstate = next_Hstate.draw('robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r');
                    current_Hstate = current_Hstate.delete_plot(); %#ok<NASGU>
                end
                if user_data_class.par.replanning == 1
                    % Here, we check if we lie in the valid linearization
                    % region of node controller or not
                    error('not updated yet')
                    %                     signed_elem_wise_difference = next_Hstate.b.est_mean.signed_element_wise_dist(obj.PRM_node);
                    %                     is_in_end_node_lnr_domain = all(abs(signed_elem_wise_difference) < user_data_class.par.valid_linearization_domain); % never forget the "absolute value operator" in computing distances.
                    %                     lost = ~is_in_end_node_lnr_domain;
                else
                    lost = 0; % By this, we indeed disable the replanning
                end
                
%                 ?????????????
            end
            
            
            absolute_landed_node_ind = (obj.PRM_orbit_number-1)*length(obj.PRM_nodes_on_orbit)+landed_node_ind_on_orbit;
            
            axis(old_limits);
        end
        function [next_hstate, lost, failed] = execute_with_replanning(obj,current_hstate,current_GHb,convergence_time) %#ok<STOUT,MANU,INUSD>
            error('Not implemented yet');
        end
    end
    
    methods (Access = private)
        function [updated_PHb , landed_node_ind_on_orbit] = stopping_condition_check_for_PHb(obj,PHb,time,convergence_time,landed_node_ind_on_orbit)
            if ~exist('landed_node_ind_on_orbit','var')
                landed_node_ind_on_orbit = [];
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
                T = obj.PRM_orbit.period;
                cyclic_time = mod(time , T)+(T*(mod(time , T)==0)); % we apply the periodic nature to the time evolution. IMPORTANT: the reason we use "k-1" instead of "k" is that we count the starting location of orbit twice when we concatenate edge and orbit. This may be revised later.
                matched_node_ind_on_orbit = find(obj.PRM_orbit.node_time_stages == cyclic_time); % here we check to see if the current time is the same as any of the node times on orbit or not.
                if ~isempty(matched_node_ind_on_orbit) % Here, we check if it is even possible to reach the FIRM node or not at this time stage (due to the determinicity of the covariance evolution under PLQG).
                    for q = alive_particles' % note that "alive_particles" vector has to be a row vector for this line to work properly. NOT a column vector.
                        bel = PHb.Hparticles(q).b;
                        candidate_FIRM_node = obj.reachable_FIRM_nodes(matched_node_ind_on_orbit);
                        if candidate_FIRM_node.is_reached(bel)  % Here, we check if the belief enters the stopping region or not.
                            updated_PHb.stopped_particles(q) = 1;
                            updated_PHb.stopping_times(q) = time;
                            landed_node_ind_on_orbit(q) = matched_node_ind_on_orbit;  % q-th entry of "landed_node" tells that which node the q-th particle has landed in.
                        end
                    end
                end
            end
        end
        function [next_Hstate, next_GHb, landed_node_ind_on_orbit] = execute_ensemble_FIRM(obj,current_Hstate,current_GHb,convergence_time)
            
            current_Hstate.stopped_particles = zeros(current_Hstate.num_p,1); % reset the stopped particles for the new node.

            while ~all_prtcls_stopped

                if k > convergence_time
                    [next_Hstate , landed_node_ind_on_orbit] = obj.stopping_condition_check_for_PHb(next_Hstate,k+1,convergence_time);
                    all_prtcls_stopped = all(next_Hstate.stopped_particles | next_Hstate.collided_particles | next_Hstate.unsuccessful_particles); % indicates if all particles are stopped or collided.
                    if show_just_once == 1
                        xlabel('Hbelief has converged. Particles  are trying to reach stopping region...')
                        disp('Hbelief has converged. Particles are trying to reach stopping region...')
                        show_just_once = 0;
                    end
                else % before convergence happens, we propagate the GHb also (only for plotting purposes)
                    next_GHb = obj.controller.propagate_Hb_Gaussian(current_GHb);
                    next_GHb = next_GHb.draw();
                    current_GHb = current_GHb.delete_plot();
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
                %                 current_GHb = next_GHb; we have commented this line, becausae we do not implement GHb for periodic case.
                k = k+1;
            end
        end
        function [next_hstate, lost, failed] = execute_with_replanning_FIRM(obj,current_hstate,current_GHb,convergence_time) %#ok<STOUT,MANU,INUSD>
            error('Not implemented yet');
        end
    end
end