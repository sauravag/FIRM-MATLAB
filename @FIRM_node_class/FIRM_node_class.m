classdef FIRM_node_class
    % This class encapsulates the "node" concept in FIRM
    % (Feedback-controller-based Information-state RoadMap).
    properties
        center_b; % Center of the FIRM node in belief space
        center_GHb; % Center of the FIRM node in Hyper-belief space.
        PRM_node; % Corresponding PRM node
        number; % Absolute number of node % Note that "node number" is not assigned in the constructor as it is not needed in general (it is the duty of FIRM graph to keep track of node numbers.) However if it is needed this property can be set directly.
        outgoing_edges = []; % Edges going out of node % Note that "outgoing_edges" is not assigned in the constructor as it is not needed in general (it is the duty of FIRM graph to keep track of node numbers.) However if it is needed this property can be set directly.
        par; % FIRM node parameters, defined by the user
    end
    properties (Access = private)
        plot_handle; % plot handle of the FIRM node
    end
    
    methods
        function obj = FIRM_node_class( node_center_inp )
            if nargin>0
                obj.center_b = node_center_inp;
                obj.par = user_data_class.par.FIRM_node_parameters;  % Note that if you are loading a saved version of FIRM graph, this "par" will be the same as the saved one and NOT the same as you current parameters.
            end
        end
        function obj = draw(obj)
            obj.center_b = obj.center_b.draw('EllipseSpec','-k','EllipseWidth',4,'color','k');
            % you can draw the neighborhood also here. Dont forget to add
            % its plot handle to "obj.plot_handle "
        end
        function obj = delete_plot(obj)
            obj.center_b = obj.center_b.delete_plot();
            delete(obj.plot_handle);
            obj.plot_handle = [];
        end
        function PHb = sample(obj,num_samples)
            mean = obj.center_b.est_mean.val;
            cov = obj.center_b.est_cov;
            sym_cov = (cov+cov')/2; % symmetric covariance matrix
            if max(max(abs(cov-sym_cov)))>1e-6
                error('covariance matrix is too unsymmetric')
            else
                cov = sym_cov;
            end
            Xg_particles = mvnrnd(mean,cov,num_samples)';
            Hs_particles = Hstate.empty;
            Hs_particles(num_samples,1) = Hstate;
            for i = 1:num_samples
                Hs_particles(i) = Hstate(state(Xg_particles(:,i)),obj.center_b);
            end
            PHb = Hbelief_p(Hs_particles , num_samples);
            PHb.num_p = num_samples;
        end
        function ssPHb = sample_stationary_GHb(obj) %#ok<MANU,STOUT>
            error('Not updated yet.')
            ssPHb = obj.center_GHb.HbeliefG_to_HbeliefP(user_data_class.par.par_n);
        end
        function YesNo = is_reached(obj,b)
            % In this function we check if the belief b has been reached to the FIRM node or not.
            
            % We first need to characterize the belief stopping region.
            % size of the region:
            X_reg_size = user_data_class.par.FIRM_node_parameters.mean_neighborhood_size; % read the comment after the "obj.par" initialization, to see why I have written the current line.
            Pest_reg_size = user_data_class.par.FIRM_node_parameters.cov_neighborhood_size;
            % center of the stopping region
            node_PRM = obj.center_b.est_mean.val;
            Pest_ss = obj.center_b.est_cov;
            % Here, we retrieve where the current belief is.
            Xest_mean = b.est_mean;
            Pest = b.est_cov;
            % compute the absolute difference from the center
            signed_Xest_diff = Xest_mean.signed_element_wise_dist(node_PRM); % this basically computes the "signed element-wise distance" between "Xest_mean" and "node_PRM"
            Xest_diff = abs(signed_Xest_diff); % Neverrrr forget "abs" here.
            Pest_diff = abs(Pest - Pest_ss);
            % Check if the estimation mean and covariance are in the
            % desired regions or not.
            in_reg_X  = all(Xest_diff < X_reg_size);
            in_reg_P  = all(all(Pest_diff < Pest_reg_size));
%             YesNo = in_reg_P && in_reg_X;
            
            % using hellinger distance
            P = 0.5*(Pest + Pest_ss);
            hellingerDistance = (1/8)*(node_PRM - Xest_mean.val)'*inv(P)*(node_PRM - Xest_mean.val)+...
                0.5*log(det(P)/sqrt(det(Pest)*det(Pest_ss)));
            disp('----------------')
            disp([Pest(1,1),Pest(2,2),Pest(3,3)])
            disp([Pest_ss(1,1),Pest_ss(2,2),Pest_ss(3,3)])
            disp('********')
            disp([node_PRM(1),node_PRM(2),node_PRM(3)])
            disp([Xest_mean.val(1),Xest_mean.val(2),Xest_mean.val(3)])
            disp(norm([Xest_mean.val(1),Xest_mean.val(2),Xest_mean.val(3)] - [node_PRM(1),node_PRM(2),node_PRM(3)]))
            disp(norm([Pest(1,1),Pest(2,2),Pest(3,3)] - [Pest_ss(1,1),Pest_ss(2,2),Pest_ss(3,3)]))
            disp(hellingerDistance)
           YesNo = hellingerDistance <= 1.5; 
        end
        function YesNo = is_GHb_converged(obj,GHb)
            % In this function we check if the Gaussian Hyper-belief GHb
            % has converged yet or not. - Obviously, this function only is
            % useful if the GHb is defined for the controller of choice.
            
            % size of convergence region
            BigX_reg_size = obj.par.GHb_conv_BigX_thresh; % distance threshold for both Xg_mean and Xest_mean_mean in the single vector
            Pest_reg_size = obj.par.GHb_conv_Pest_thresh; % defines the convergence threshold for Pest
            BigCov_reg_size = obj.par.GHb_conv_BigCov_thresh; % defines the convergence threshold for BigCov
            % center of convergence region
            node_PRM = obj.center_GHb.Xg_mean.val;
            BigXmean_center = [node_PRM;node_PRM];
            Pest_center = obj.center_GHb.Pest;
            BigCov_center = obj.center_GHb.P_of_joint;
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
    end
    
end