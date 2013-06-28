%% Class Definition
classdef Node_DFL_EKF < FIRM_node_interface
    %==============================  FIRM node based on Nonlinear Unicycle Controller (Oriolo's paper) and EKF =========================================
    % This class encapsulates the "node" concept in FIRM
    % (Feedback-controller-based Information-state RoadMap). This node is
    % based on Nonlinear Unicycle Controller (Oriolo's paper) and EKF 
    properties
        node_center; % This is a point in belief space, around which the stopping region is defined.
        node_controller; % This is the nonlinear controller (based on oriolo's paper) designed for the unicycle.
        PRM_node; % Underlying PRM node
        number; % The FIRM node number in the entire FIRM graph
        plot_handle;
    end
    properties (Constant = true)
        FIRM_stop_reg = get_stopping_region(); % Stopping region in this nonlinear cotroller-based node. 
        % The "get_stopping_region" function is defined out of "methods" block.
    end
    
    methods
        function obj = Node_DFL_EKF(PRM_node_inp,node_number_inp)
            if nargin>0
                obj.number = node_number_inp;
                obj.PRM_node = PRM_node_inp;
                obj.node_controller = DFL_EKF_class(PRM_node_inp.val); % Note that the node controller is an object of "DFL_EKF_class" NOT simple "DFL_EKF_class" itself.
            end
        end
        function obj = construct_node(obj)
            if isempty(obj.PRM_node)
               error('Ali: The "PRM_node" has not provided in node construction.')
            end
            obj.node_center = belief(obj.PRM_node, obj.node_controller.Stationary_Pest);
        end
        function obj = draw(obj)
            obj.node_center = obj.node_center.draw();
        end
        function [seq_of_PHb, seq_of_GHb] = construct_seq_of_Hb(obj,initial_PHb, initial_GHb)
            if ~user_data_class.par.No_plot
                % Zoom in to the node area in the figure
                old_zoom = obj.zoom_in();
                xlabel(['Node controller of node ',num2str(obj.number),' is working ...']);
            end
            % in following "if" condition, we choose between two functions
            % that construct the sequences for HBRM or FIRM.
            if strcmpi(user_data_class.par.RoadMap,'HBRM')
                [seq_of_PHb, seq_of_GHb] = obj.construct_seq_of_Hb_for_HBRM(initial_PHb, initial_GHb);
            elseif strcmpi(user_data_class.par.RoadMap,'FIRM')
                [seq_of_PHb, seq_of_GHb] = obj.construct_seq_of_Hb_for_FIRM(initial_PHb, initial_GHb);
            end
            % zoom out
            if ~user_data_class.par.No_plot
                obj.zoom_out(old_zoom);
            end
        end
        function ssPHb = sample_stationary_GHb(obj)
            % in the following we construct the GHb!!! but it does not
            % exists in nonlinear controller case!!! So it is actually a
            % dirac delta since we are in the epsilon neighborhood of the
            % "node_center". Following code should be removed or fixed
            % ASAP.
            Xg_mean = obj.node_center.est_mean;
            Xest_MeanOfMean = Xg_mean;
            Pest = obj.node_center.est_cov;
            stdim = state.dim;
            P_of_joint = [Pest , zeros(stdim,stdim)  ;  zeros(stdim,stdim) , eps*eye(stdim) ];
            
            initial_GHb = Hbelief_G(Xg_mean, Xest_MeanOfMean, Pest, P_of_joint);
              
            ssPHb = initial_GHb.HbeliefG_to_HbeliefP(user_data_class.par.par_n);
        end
        function [next_ensemble, next_GHb] = execute_ensemble(obj,current_ensemble,current_GHb,convergence_time)
            % Zoom in to the node area in the figure
            old_zoom = obj.zoom_in();
            xlabel(['Node controller of ',user_data_class.par.RoadMap,' node ',num2str(obj.number),' is working ...']);
            if strcmpi(user_data_class.par.RoadMap,'HBRM')
                [next_ensemble, next_GHb] = obj.execute_ensemble_HBRM(current_ensemble,current_GHb,convergence_time);
            elseif strcmpi(user_data_class.par.RoadMap,'FIRM')
                [next_ensemble, next_GHb] = obj.execute_ensemble_FIRM(current_ensemble,current_GHb,convergence_time);
            end
            obj.zoom_out(old_zoom);
        end
        function [next_hstate, lost, failed] = execute_single_robot_with_replanning(obj,current_hstate,current_GHb,convergence_time)
            % Zoom in to the node area in the figure
            old_zoom = obj.zoom_in();
            xlabel(['Node controller of ',user_data_class.par.RoadMap,' node ',num2str(obj.number),' is working ...']);
            if strcmpi(user_data_class.par.RoadMap,'HBRM')
                [next_hstate, lost, failed] = obj.execute_with_replanning_HBRM(current_hstate,convergence_time);
            elseif strcmpi(user_data_class.par.RoadMap,'FIRM')
                [next_hstate, lost, failed] = obj.execute_with_replanning_FIRM(current_hstate,current_GHb,convergence_time);
            end
            obj.zoom_out(old_zoom);
        end
    end
    
    methods (Access = private)
        function updated_PHb = stopping_condition_check_for_PHb(obj,PHb,time,convergence_time)
            % This function updates the "stopped_particles" and "unsuccessful_particles" properties of the PHb at time step k+1.
            updated_PHb = PHb; % we preserve all the information in PHb and only update the its "stopped_particles" property.
            alive_particles = find(~PHb.stopped_particles & ~PHb.collided_particles); % alive particles are those that have not collided or stopped yet.
            % if the time is greater than the maximum allowed time, alive 
            % particles are considered as the unsuccessful particles.
            if time - convergence_time > user_data_class.par.max_stopping_time
                updated_PHb.unsuccessful_particles(alive_particles) = 1;
                updated_PHb.stopping_times(alive_particles) = time;
            else % if the time is OK, for alive particles, we check if the 
                % stopping condition is satisfied or not
                for q = alive_particles' % note that "alive_particles" vector has to be a row vector for this line to work properly. NOT a column vector.
                    bel = PHb.Hparticles(q).b;
                    if is_belief_in_stop_region(obj,bel) % Here, we check if the belief enters the stopping region or not.
                        updated_PHb.stopped_particles(q) = 1;
                        updated_PHb.stopping_times(q) = time;
                    end
                end
            end
        end
        function YesNo = is_belief_in_stop_region(obj,b)
            % In this function we check if the belief b is in the stopping
            % region or not.
            % This stopping condition is used in FIRM.
            
            % We first need to characterize the belief stopping region.
            % size of the region:
            X_reg_size = obj.FIRM_stop_reg.X_thresh;
            Pest_reg_size = obj.FIRM_stop_reg.Pest_thresh;
            % center of the stopping region
            node_PRM = obj.node_center.est_mean.val;
            Pest_ss = obj.node_center.est_cov;
            % Here, we retrieve where the current belief is.
            Xest_mean = b.est_mean.val;
            Pest = b.est_cov;
            % compute the absolute difference from the center
            Xest_diff = abs(Xest_mean - node_PRM); % Do not forget "abs" here.
            Pest_diff = abs(Pest - Pest_ss);
            % Check if the estimation mean and covariance are in the
            % desired regions or not.
            in_reg_X  = all(Xest_diff < X_reg_size);    
            in_reg_P  = all(all(Pest_diff < Pest_reg_size));
            YesNo = in_reg_P && in_reg_X;
        end
        function YesNo = Is_GHb_converged(obj,GHb)
           % size of convergence region
           BigX_reg_size = obj.HBRM_converg_reg.BigX_thresh; % distance threshold for both Xg_mean and Xest_mean_mean in the single vector
           Pest_reg_size = obj.HBRM_converg_reg.Pest_thresh; % defines the convergence threshold for Pest
           BigCov_reg_size = obj.HBRM_converg_reg.BigCov_thresh; % defines the convergence threshold for BigCov
           % center of convergence region
           node_PRM = obj.stationary_GHb.Xg_mean.val;
           BigXmean_center = [node_PRM;node_PRM];
           Pest_center = obj.stationary_GHb.Pest;
           BigCov_center = obj.stationary_GHb.P_of_joint;
           % retrieve where we are
           BigX_mean = [GHb.Xg_mean.val;GHb.Xest_mean_mean.val];
           Pest = GHb.Pest;
           BigCov = GHb.P_of_joint;
           % compute the absolute distances from the center
           BigX_dist = abs(BigX_mean - BigXmean_center);
           Pest_dist = abs(Pest - Pest_center);
           BigCov_dist = abs(BigCov - BigCov_center);
           % Check if the distances are in the convergence thresholds or
           % not
           in_reg_BigX = all(BigX_dist < BigX_reg_size);
           in_reg_Pest = all(all(Pest_dist < Pest_reg_size));
           in_reg_BigCov = all(all(BigCov_dist < BigCov_reg_size));
           % Approximate convergence happens if all BigX and Pest and
           % BigCov are in their corresponding convergence regions.
           YesNo = in_reg_BigX && in_reg_Pest && in_reg_BigCov;
        end
        function old_limits = zoom_in(obj)
            error('you have to shift the zoom_in to the state class')
            old_xlim = xlim;
            old_ylim = ylim;
            old_limits = [old_xlim,old_ylim];
            zoom_ratio = 3;
            new_center = obj.node_center.est_mean.val;
            new_x_length = (old_xlim(2)-old_xlim(1))/zoom_ratio;
            new_y_length = (old_ylim(2)-old_ylim(1))/zoom_ratio;
            new_xlim = new_center(1) + [-new_x_length,new_x_length]/2;
            new_ylim = new_center(2) + [-new_y_length,new_y_length]/2;
            axis([new_xlim,new_ylim])
        end
        function zoom_out(obj,old_limits) %#ok<MANU>
            error('There should be no zoom_out function'
            axis(old_limits);
        end
        function [seq_of_PHb, seq_of_GHb] = construct_seq_of_Hb_for_FIRM(obj,initial_PHb, initial_GHb)
            disp(['Node controller of FIRM node ',num2str(obj.number),' is working ...'])
            % initializing the particle Hbelief sequence
            seq_of_PHb = Hbelief_p.empty; % Without this line we get the "object conversion error to double". There should be a better way of overcoming this problem, but I do not know at this time.
            seq_of_PHb(1) = initial_PHb;
            % initializing the Gaussian Hbelief sequence
            seq_of_GHb = Hbelief_G.empty; % Without this line we get the "object conversion error to double". There should be a better way of overcoming this problem, but I do not know at this time.
            seq_of_GHb(1) = initial_GHb;
            
            k = 1; % k denotes the time step
            disp('AliFW: Following memory preallocatoins are too conservative. Fix it please.')
            seq_of_PHb(1,2*user_data_class.par.max_stopping_time) = Hbelief_p; % memory preallocation for PHb. We allocate the twice of the "maximum particle stopping time", because, usually the convergence time is less than "maximum particle stopping time". Note that we will cut this sequence after the exact length of sequence is found.
            seq_of_GHb(1,2*user_data_class.par.max_stopping_time) = Hbelief_G; % memory preallocation for PHb. We allocate the twice of the "maximum particle stopping time", because, usually the convergence time is less than "maximum particle stopping time". Note that we will cut this sequence after the exact length of sequence is found.
            
            % following loop propagates both PHb and GHb until the stopping
            % condition is satisfied.
            all_prtcls_stopped = 0; % initial value
            convergence_happend = 0; % initial value
            convergence_time = '??'; % initilization. For display only.
            global Previous_control;
            Previous_control = zeros(MotionModel_class.ctDim , 1);  % Since we are using a "Dynamic feedback linearization based" controller, at each step
            % we need to have the prevous control signal value. So, every
            % time we are starting a new edge, we have to initialize it be
            % zero here.
            while ~all_prtcls_stopped
                disp(['Step ',num2str(k),' of FIRM node-controller ',num2str(obj.number),'. Hb has convereged at ',num2str(convergence_time),' and max stopping time after that is ',num2str(user_data_class.par.max_stopping_time),' .']);
                % in following we compute the sequence of Particle-Hbelief
                % induced by the node controller (stationary-LQG)
                seq_of_PHb(k+1) = obj.node_controller.propagate_Hb_particle(seq_of_PHb(k));
                if ~user_data_class.par.No_plot
                    seq_of_PHb(k+1) = seq_of_PHb(k+1).draw('robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r');
                    seq_of_PHb(k) = seq_of_PHb(k).delete_plot();
                    drawnow
                end
                
                % We start to check if FIRM stopping condition is satisfied or not.
                
                % Following function check if the particles enter the
                    % stopping region or not. Moreover, it set the
                    % "stopping_times" property of the "Hbelief_p" to its
                    % corresponding value. Therefore, we provide the time
                    % k+1 as well, among input arguments.
                    seq_of_PHb(k+1) = obj.stopping_condition_check_for_PHb(seq_of_PHb(k+1),k+1,convergence_time);  % Note that both input and output are in time step k+1. This function updates the "stopped_particles" and "unsuccessful_particles" properties of the PHb at time step k+1.
                    all_prtcls_stopped = all(seq_of_PHb(k+1).stopped_particles | seq_of_PHb(k+1).collided_particles | seq_of_PHb(k+1).unsuccessful_particles); % indicates if all particles are stopped or collided.
                
                    k = k+1; % progress the time
            end
            seq_of_PHb = seq_of_PHb(1:k); % Here we cut the extra part of "seq_of_PHb".
        end
        function [seq_of_PHb, seq_of_GHb] = construct_seq_of_Hb_for_HBRM(obj,initial_PHb,initial_GHb)
            % we do not propagate the PHb in HBRM. Thus, we do not need
            % "initial_PHb" and put "~" in input argument list.
            disp(['Node controller of HBRM node ',num2str(obj.number),' is working ...'])
            % initializing the Gaussian Hbelief sequence
            seq_of_GHb = Hbelief_G.empty; % Without this line we get the "object conversion error to double". There should be a better way of overcoming this problem, but I do not know at this time.
            seq_of_GHb(1) = initial_GHb;
            seq_of_PHb = Hbelief_p.empty;
            seq_of_PHb(1) = initial_PHb;
            
            k = 1; % k denotes the time step
            disp('AliFW: Following memory preallocatoins are too conservative. Fix it please.')
            seq_of_GHb(1,2*user_data_class.par.max_stopping_time) = Hbelief_G; % memory preallocation for PHb. We allocate the twice of the "maximum particle stopping time", because, usually the convergence time is less than "maximum particle stopping time". Note that we will cut this sequence after the exact length of sequence is found.
            
            % following loop propagates the GHb until the stopping
            % condition is satisfied.
            convergence_happend = 0; % initial value
            while ~convergence_happend
                disp(['Step ',num2str(k),' of HBRM node-controller ',num2str(obj.number)]);
                % in following we compute the sequence of Gaussian-Hbelief
                % induced by the node controller (stationary-LQG)
                    seq_of_GHb(k+1) = obj.node_controller.propagate_Hb_Gaussian(seq_of_GHb(k));
                    if ~user_data_class.par.No_plot
                        seq_of_GHb(k+1) = seq_of_GHb(k+1).draw();
                        seq_of_GHb(k) = seq_of_GHb(k).delete_plot();
                        drawnow
                    end
                    % check if the GHb is converged to the stationary GHb of
                    % the node or not.
                    convergence_happend = obj.Is_GHb_converged(seq_of_GHb(k+1));
                    if convergence_happend == 1
                        convergence_time = k+1; % we do not output this variable anymore, because the length of following sequence is the same as this variable.
                        seq_of_GHb = seq_of_GHb(1:k+1); % Here we cut the extra part of the "seq_of_GHb".
                        xlabel(['Hbelief converged at step ',num2str(convergence_time),' .'])
                    end
                    % In construction phase we do not need to propagate the PHb in
            % the "node-controller". However, for plotting purposes, we do
            % it only if we want to plot something.
            if ~user_data_class.par.No_plot
                seq_of_PHb(k+1) = obj.node_controller.propagate_Hb_particle(seq_of_PHb(k));
                seq_of_PHb(k+1) = seq_of_PHb(k+1).draw();
                seq_of_PHb(k) = seq_of_PHb(k).delete_plot();
            else
                seq_of_PHb = 'No PHb in construction phase in HBRM node!'; % in HBRM there no "seq_of_PHb" is produced by the node-controller.
            end
                k = k+1; % progress the time
            end
            
        end
        function [next_ensemble, next_GHb] = execute_ensemble_FIRM(obj,current_ensemble,current_GHb,convergence_time)
            show_just_once = 1;
            k = 1;
            current_ensemble.stopped_particles = zeros(Hbelief_p.num_p,1); % reset the stopped particles for the new node.
            all_prtcls_stopped = 0; % initialization
            while ~all_prtcls_stopped
                disp(['Step ',num2str(k),' of node-controller ',num2str(obj.number),'. convergence time is ',num2str(convergence_time),'.']);
                next_ensemble = obj.node_controller.propagate_Hb_particle(current_ensemble);
                next_ensemble = next_ensemble.draw('robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r');
                current_ensemble = current_ensemble.delete_plot(); %#ok<NASGU>
                if k > convergence_time
                    next_ensemble = obj.stopping_condition_check_for_PHb(next_ensemble,k+1,convergence_time);
                    all_prtcls_stopped = all(next_ensemble.stopped_particles | next_ensemble.collided_particles | next_ensemble.unsuccessful_particles); % indicates if all particles are stopped or collided.
                    if show_just_once == 1
                        xlabel('Hbelief has converged. Particles  are trying to reach stopping region...')
                        disp('Hbelief has converged. Particles are trying to reach stopping region...')
                        show_just_once = 0;
                    end
                else % before convergence happens, we propagate the GHb also (only for plotting purposes)
                    next_GHb = obj.node_controller.propagate_Hb_Gaussian(current_GHb);
                    next_GHb = next_GHb.draw();
                    current_GHb = current_GHb.delete_plot(); %#ok<NASGU>
                end
                drawnow
                % making a video of runtime
                if user_data_class.par.sim.video == 1
                    global vidObj; %#ok<TLEV>
                    currFrame = getframe(gcf);
                    writeVideo(vidObj,currFrame);
                end
                % update the ensemble and GHb
                current_ensemble = next_ensemble;
                current_GHb = next_GHb;
                k = k+1;
            end
        end
        function [next_ensemble, next_GHb] = execute_ensemble_HBRM(obj,current_ensemble,current_GHb,convergence_time)
            for k = 1:convergence_time
                disp(['Step ',num2str(k),' of HBRM node-controller ',num2str(obj.number),'. convergence time is ',num2str(convergence_time),'.']);
                % propagation of ensemble
                next_ensemble = obj.node_controller.propagate_Hb_particle(current_ensemble);
                next_ensemble = next_ensemble.draw('robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r');
                current_ensemble = current_ensemble.delete_plot(); %#ok<NASGU>
                % propagation of GHb
                next_GHb = obj.node_controller.propagate_Hb_Gaussian(current_GHb);
                next_GHb = next_GHb.draw();
                current_GHb = current_GHb.delete_plot(); %#ok<NASGU>
                drawnow
                % making a video of runtime
                if user_data_class.par.sim.video == 1
                    global vidObj; %#ok<TLEV>
                    currFrame = getframe(gcf);
                    writeVideo(vidObj,currFrame);
                end
                % update the ensemble and GHb
                current_ensemble = next_ensemble;
                current_GHb = next_GHb;
            end
        end
        function [next_hstate, lost, failed] = execute_with_replanning_HBRM(obj,current_hstate,convergence_time)
            failed = 0; % we assume no collision in node region.
            for k = 1:convergence_time
                disp(['Step ',num2str(k),' of HBRM node-controller ',num2str(obj.number),'. convergence time is ',num2str(convergence_time),'.']);
                next_hstate = obj.node_controller.propagate_Hstate(current_hstate);
                next_hstate = next_hstate.draw();
                current_hstate = current_hstate.delete_plot(); %#ok<NASGU>
                drawnow
                % making a video of runtime
                if user_data_class.par.sim.video == 1
                    global vidObj; %#ok<TLEV>
                    currFrame = getframe(gcf);
                    writeVideo(vidObj,currFrame);
                end
                % Here, we check if we lie in the valid linearization
                % region of node controller or not
                is_in_end_node_lnr_domain = all(abs(next_hstate.b.est_mean.val - obj.PRM_node.val) < user_data_class.par.valid_linearization_domain); % never forget the "absolute value operator" in computing distances.
                lost = ~is_in_end_node_lnr_domain;
                if lost
                    return
                end
                % update the ensemble
                current_hstate = next_hstate;
%               next_GHb = obj.node_controller.propagate_Hb_Gaussian(current_GHb);
%               next_GHb = next_GHb.draw();
%               current_GHb = current_GHb.delete_plot();
            end
        end
        function [next_hstate, lost, failed] = execute_with_replanning_FIRM(obj,current_hstate,current_GHb,convergence_time)
            show_just_once = 1;
            k = 1;
            particle_stopped = 0;
%             current_ensemble.stopped_particles = zeros(Hbelief_p.num_p,1); % reset the stopped particles for the new node.
%             all_prtcls_stopped = 0; % initialization
            while ~particle_stopped
                disp(['Step ',num2str(k),' of FIRM node-controller ',num2str(obj.number),'. convergence time is ',num2str(convergence_time),'.']);
                next_hstate = obj.node_controller.propagate_Hstate(current_hstate);
                next_hstate = next_hstate.draw();
                current_hstate = current_hstate.delete_plot(); %#ok<NASGU>
                
                % Here, we check if we lie in the valid linearization
                % region of node controller or not
                is_in_end_node_lnr_domain = all(abs(next_hstate.b.est_mean.val - obj.PRM_node.val) < user_data_class.par.valid_linearization_domain); % never forget the "absolute value operator" in computing distances.
                lost = ~is_in_end_node_lnr_domain;
                if lost
                    failed = 0;
                    return
                end
                
                if k > convergence_time + user_data_class.par.max_stopping_time
                    failed = 1;
                    lost = 0;
                    return
                elseif k > convergence_time
                    if is_belief_in_stop_region(obj,next_hstate.b) % Here, we check if the belief enters the stopping region or not.
                        failed = 0;
                        lost = 0;
                        particle_stopped = 1;
                    end
                    if show_just_once == 1 % we do not want to update the xlabel at each step.
                        xlabel('Hbelief has converged. Particles  are trying to reach stopping region...')
                        disp('Hbelief has converged. Particles are trying to reach stopping region...')
                        show_just_once = 0;
                    end
                else % before convergence happens, we propagate the GHb also (only for plotting purposes)
                    next_GHb = obj.node_controller.propagate_Hb_Gaussian(current_GHb);
                    next_GHb = next_GHb.draw();
                    current_GHb = current_GHb.delete_plot(); %#ok<NASGU>
                end
                drawnow
                % making a video of runtime
                if user_data_class.par.sim.video == 1
                    global vidObj; %#ok<TLEV>
                    currFrame = getframe(gcf);
                    writeVideo(vidObj,currFrame);
                end
                % update the ensemble and GHb
                current_hstate = next_hstate;
                current_GHb = next_GHb;
                k = k+1;
            end
        end
    end
end

%% Computing the thresholds for characterizing the stopping region
function stop_reg = get_stopping_region()
    thresh_meter = user_data_class.par.stop_region_thresh_meter;
    thresh_rad = user_data_class.par.stop_region_thresh_rad;
    X_thresh = [thresh_meter;thresh_meter;thresh_rad];
    half_Pest_thresh = X_thresh;
    Pest_thresh = half_Pest_thresh*half_Pest_thresh';
    stop_reg.X_thresh = X_thresh;   % The maximum deviation from the mean (center of stopping region)
    stop_reg.Pest_thresh = Pest_thresh;  % % The maximum deviation from the covarinace (center of stopping region)
end
% function HBRM_conv_reg = get_convergence_region()
%     Xthresh = user_data_class.par.GHb_convergence_reg_thresh'; % distance threshold for either Xg_mean or Xest_mean_mean; Xdist has to be a column vector
%     BigX_thresh = [Xthresh;Xthresh];
%     HBRM_conv_reg.BigX_thresh = BigX_thresh; % distance threshold for both Xg_mean and Xest_mean_mean in the single vector
%     HBRM_conv_reg.Pest_thresh = Xthresh*Xthresh'; % defines the convergence threshold for Pest
%     HBRM_conv_reg.BigCov_thresh = BigX_thresh*BigX_thresh'; % defines the convergence threshold for BigCov
% end