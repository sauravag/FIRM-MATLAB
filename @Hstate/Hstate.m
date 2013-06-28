classdef Hstate
    properties
        Xg; % ground truth state
        b; % belief
    end
    
    methods
        function obj = Hstate(arg1,arg2)
            if nargin == 1
                obj = arg1;
            elseif nargin == 2
                obj.Xg = arg1;
                obj.b = arg2;
            end
        end
        function obj = draw(obj, varargin)
            % The full list of properties for this function is:
            % 'RobotShape', 'RobotSize', 'XgTriaColor', 'XgColor', 
            % 'XestTriaColor', 'XestColor', 'HeadShape',
            % 'HeadSize', 'EllipseSpec'.
            if isempty(varargin)
                % defaule properties for drawing "Hstate".
                varargin = {'robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','r'};
            end
            New_varargin = varargin;
            tmp_ind = find(strcmpi(New_varargin ,'XgColor'));
            if ~isempty(tmp_ind), New_varargin{tmp_ind} = 'Color'; end;
            tmp_ind = find(strcmpi(New_varargin ,'XgTriaColor'));
            if ~isempty(tmp_ind), New_varargin{tmp_ind} = 'TriaColor'; end; 
            tmp_ind = find(strcmpi(New_varargin ,'XestColor') | strcmpi(New_varargin ,'XestTriaColor') | strcmpi(New_varargin ,'EllipseSpec') | strcmpi(New_varargin ,'EllipseWidth'));
            if ~isempty(tmp_ind)
                New_varargin([tmp_ind,tmp_ind+1]) = [];
            end
            obj.Xg = obj.Xg.draw(New_varargin{:});
            
            New_varargin = varargin;
            tmp_ind = find(strcmpi(New_varargin ,'XestColor'));
            if ~isempty(tmp_ind), New_varargin{tmp_ind} = 'Color'; end;
            tmp_ind = find(strcmpi(New_varargin ,'XestTriaColor'));
            if ~isempty(tmp_ind), New_varargin{tmp_ind} = 'TriaColor'; end;
            tmp_ind = find(strcmpi(New_varargin ,'XgColor') | strcmpi(New_varargin ,'XgTriaColor'));
            if ~isempty(tmp_ind)
                New_varargin([tmp_ind,tmp_ind+1]) = [];
            end
            tmp=get(gca,'NextPlot'); hold on
            obj.b = obj.b.draw(New_varargin{:});
            set(gca,'NextPlot',tmp);
        end
        function obj = delete_plot(obj, varargin)
            obj.Xg = obj.Xg.delete_plot();
            obj.b = obj.b.delete_plot();
        end
        function obj = draw_CovOnNominal(obj, nominal_state, varargin)
            % This function draws the Hstate. However, the estimation
            % covarinace is centered at the nominal state, provided by the
            % function caller.
            if isempty(varargin)
                % defaule properties for drawing "Hstate".
                varargin = {'robotshape','triangle','XgtriaColor','g','XestTriaColor','r','XgColor','g','XestColor','b'};
            end
            New_varargin = varargin;
            tmp_ind = find(strcmpi(New_varargin ,'XgColor'));
            if ~isempty(tmp_ind), New_varargin{tmp_ind} = 'Color'; end;
            tmp_ind = find(strcmpi(New_varargin ,'XgTriaColor'));
            if ~isempty(tmp_ind), New_varargin{tmp_ind} = 'TriaColor'; end; 
            tmp_ind = find(strcmpi(New_varargin ,'XestColor') | strcmpi(New_varargin ,'XestTriaColor') | strcmpi(New_varargin ,'EllipseSpec') | strcmpi(New_varargin ,'EllipseWidth'));
            if ~isempty(tmp_ind)
                New_varargin([tmp_ind,tmp_ind+1]) = [];
            end
            obj.Xg = obj.Xg.draw(New_varargin{:});
            
            New_varargin = varargin;
            tmp_ind = find(strcmpi(New_varargin ,'XestColor'));
            if ~isempty(tmp_ind), New_varargin{tmp_ind} = 'Color'; end;
            tmp_ind = find(strcmpi(New_varargin ,'XestTriaColor'));
            if ~isempty(tmp_ind), New_varargin{tmp_ind} = 'TriaColor'; end;
            tmp_ind = find(strcmpi(New_varargin ,'XgColor') | strcmpi(New_varargin ,'XgTriaColor'));
            if ~isempty(tmp_ind)
                New_varargin([tmp_ind,tmp_ind+1]) = [];
            end
            tmp=get(gca,'NextPlot'); hold on
            obj.b = obj.b.draw_CovOnNominal(nominal_state , New_varargin{:});
            set(gca,'NextPlot',tmp);
        end
    end
    
end