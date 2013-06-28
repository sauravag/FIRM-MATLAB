classdef belief < belief_interface
    % This class encapsulates the Gaussian belief concept.

    methods
        function obj = belief(varargin)
            obj = obj@belief_interface(varargin{:});
        end
        function obj = draw(obj, varargin)
            % The full list of properties for this function is:
            % 'RobotShape', 'RobotSize', 'TriaColor', 'color', 'HeadShape',
            % 'HeadSize', 'EllipseSpec'.
            
            ellipse_spec = '-r';  % Default value for "EllipseSpec" property.
            ellipse_width = 1;
            ellipse_magnify = 1;
            New_varargin={};
            
            for i = 1 : 2 : length(varargin)
                switch lower(varargin{i})
                    case lower('EllipseSpec')
                        ellipse_spec = varargin{i+1};
                    case lower('EllipseWidth')
                        ellipse_width = varargin{i+1};
                    case lower('EllipseMagnify')
                        ellipse_magnify = varargin{i+1};
                    otherwise  % So, if the "varargin" is anything else, it is related to the plotting of "mean". So, we collect such inputs in "New_varargin" and pass them to the "obj.est_mean.draw".
                        New_varargin{end+1} = varargin{i}; %#ok<AGROW>
                        New_varargin{end+1} = varargin{i+1}; %#ok<AGROW> % Note that in previous line the "end" itself is increased by 1. Thus, this line is correct.
                end
            end
            obj.est_mean = obj.est_mean.draw(New_varargin{:});
           if ~isempty(ellipse_spec)
               tmp=get(gca,'NextPlot'); hold on
               obj.ellipse_handle = [];
               for j = 1:state.num_robots
                   tmp_h = plotUncertainEllip2D(obj.est_cov(3*j-2:3*j-1  ,  3*j-2:3*j-1),obj.est_mean.val(3*j-2:3*j-1), ellipse_spec, ellipse_width,ellipse_magnify);
                   obj.ellipse_handle = [obj.ellipse_handle , tmp_h];
               end
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
               for j = 1:state.num_robots
                   tmp_h = plotUncertainEllip2D(obj.est_cov(3*j-2:3*j-1  ,  3*j-2:3*j-1),nominal_state.val(3*j-2:3*j-1),ellipse_spec, ellipse_width);
                   obj.ellipse_handle = [obj.ellipse_handle , tmp_h];
               end
                set(gca,'NextPlot',tmp);
            end
        end
    end
end