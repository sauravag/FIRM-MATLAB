classdef Planar_dyn_manipulator_belief < Gaussian_belief_interface
    % This class encapsulates the Gaussian belief concept.

    methods
        function obj = Planar_dyn_manipulator_belief(varargin)
            obj = obj@Gaussian_belief_interface(varargin{:});
        end
        function obj = draw(obj, varargin)
            % The full list of properties for this function is:
            % 'RobotShape', 'RobotSize', 'TriaColor', 'color', 'HeadShape',
            % 'HeadSize', 'EllipseSpec'.
            
            ellipse_spec = '-r';  % Default value for "EllipseSpec" property.
            ellipse_width = 2;
            New_varargin={};
            
            for i = 1 : 2 : length(varargin)
                switch lower(varargin{i})
                    case lower('EllipseSpec')
                        ellipse_spec = varargin{i+1};
                    case lower('EllipseWidth')
                        ellipse_width = varargin{i+1};
                    otherwise  % So, if the "varargin" is anything else, it is related to the plotting of "mean". So, we collect such inputs in "New_varargin" and pass them to the "obj.est_mean.draw".
                        New_varargin{end+1} = varargin{i}; %#ok<AGROW>
                        New_varargin{end+1} = varargin{i+1}; %#ok<AGROW> % Note that in previous line the "end" itself is increased by 1. Thus, this line is correct.
                end
            end
            obj.est_mean = obj.est_mean.draw(New_varargin{:});
            if ~isempty(ellipse_spec)
                tmp=get(gca,'NextPlot'); hold on
                obj.ellipse_handle = [];
                
                tip_est_mean = obj.est_mean.joint_2D_locations(:,end);
                num_joints = state.num_revolute_joints;
                
                Jacobian = nan(2,num_joints);
                sum_theta = sum(obj.est_mean.val(1:num_joints));
                Jacobian(:,num_joints) = [-sin(sum_theta);cos(sum_theta)];
                for k = num_joints-1:-1:1
                    sum_theta = sum(obj.est_mean.val(1:k));
                    Jacobian(:,k) = [-sin(sum_theta);cos(sum_theta)]+Jacobian(:,k+1);
                end
                tip_cov = Jacobian * obj.est_cov(1:num_joints , 1:num_joints) * Jacobian';
                
                obj.ellipse_handle = plotUncertainEllip2D(tip_cov, tip_est_mean, ellipse_spec, ellipse_width);
                set(gca,'NextPlot',tmp);
            end
        end
        function obj = draw_CovOnNominal(obj, nominal_state, varargin)
            % This function draws the belief. However, the estimation
            % covarinace is centered at the nominal state, provided by the
            % function caller.
            
            ellipse_spec = '-r';  % Default value for "EllipseSpec" property.
            ellipse_width = 2;
            New_varargin={};
            
            for i = 1 : 2 : length(varargin)
                switch lower(varargin{i})
                    case lower('EllipseSpec')
                        ellipse_spec = varargin{i+1};
                    case lower('EllipseWidth')
                        ellipse_width = varargin{i+1};
                    otherwise  % So, if the "varargin" is anything else, it is related to the plotting of "mean". So, we collect such inputs in "New_varargin" and pass them to the "obj.est_mean.draw".
                        New_varargin{end+1} = varargin{i}; %#ok<AGROW>
                        New_varargin{end+1} = varargin{i+1}; %#ok<AGROW> % Note that in previous line the "end" itself is increased by 1. Thus, this line is correct.
                end
            end
            obj.est_mean = obj.est_mean.draw(New_varargin{:});
           if ~isempty(ellipse_spec)
               tmp=get(gca,'NextPlot'); hold on
               obj.ellipse_handle = [];
               
               num_joints = state.num_revolute_joints;
               
               Jacobian = nan(2,num_joints);
               sum_theta = sum(obj.est_mean.val(1:num_joints));
               Jacobian(:,num_joints) = [-sin(sum_theta);cos(sum_theta)];
               for k = num_joints-1:-1:1
                   sum_theta = sum(obj.est_mean.val(1:k));
                   Jacobian(:,k) = [-sin(sum_theta);cos(sum_theta)]+Jacobian(:,k+1);
               end
               tip_cov = Jacobian * obj.est_cov(1:num_joints , 1:num_joints) * Jacobian';
               
               obj.ellipse_handle = plotUncertainEllip2D(tip_cov, nominal_state.joint_2D_locations(:,end), ellipse_spec, ellipse_width);
                   
                set(gca,'NextPlot',tmp);
            end
        end
    end
end