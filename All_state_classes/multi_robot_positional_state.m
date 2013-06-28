classdef multi_robot_positional_state < state_interface
    % This class encapsulates the state of a planar robot, described by its 2D location and heading angle.
    properties (Constant)
        num_robots = user_data_class.par.state_parameters.num_robots;
        dim = 2*state.num_robots; % state dimension
    end
    properties
        val; % value of the state
        plot_handle=[]; % handle for the "drawings" associated with the state
        head_handle=[];
        tria_handle=[];
        text_handle=[]; % handle for the "displayed text" associated with the state
    end
    
    
    methods
        function obj = multi_robot_positional_state(varargin) % Note that the format (spacing and words) of this function must remain the same (as it is used in the typeDef function)
            obj = obj@state_interface(varargin{:});
        end
        function signed_dist_vector = signed_element_wise_dist(obj,x2) % this function returns the "Signed element-wise distnace" between two states x1 and x2
            x1 = obj.val; % retrieve the value of the state vector
            if isa(x2,'multi_robot_positional_state'), x2=x2.val; end % retrieve the value of the state vector  % Note that the format (spacing and words) of this function must remain the same (as it is used in the typeDef function)
            signed_dist_vector = x1 - x2;
        end
        function obj = draw(obj, varargin)
            % The full list of properties for this function is:
            % 'RobotShape', 'RobotSize', 'TriaColor', 'color', 'HeadShape',
            % 'HeadSize'.
            % default values
            robot_shape = 'triangle';
            robot_size = 0.5;
            robot_color = {'r','g','b'};
            head_shape = {'s','o','d'};
            head_size = 6;
            robot_text = {};
            font_size = 15;
            text_color  = 'b';
            for i = 1 : 2 : length(varargin)
                switch lower(varargin{i})
                    case lower('RobotShape')
                        robot_shape = varargin{i+1}; 
                    case lower('RobotSize')
                        robot_size = varargin{i+1};
                    case lower('TriaColor')
%                         robot_color = varargin{i+1};
                    case lower('color')
%                         robot_color = varargin{i+1};
                    case lower('HeadShape')
%                         head_shape = varargin{i+1};
                    case lower('HeadSize')
                        head_size = varargin{i+1};
                    case lower('text')
                        robot_text = varargin{i+1};
                    case lower('fontsize')
                        font_size = varargin{i+1};
                    case lower('textcolor')
                        text_color = varargin{i+1};
                    otherwise
                        error('This property does not exist.')
                end
            end
            
            tmp=get(gca,'NextPlot');
            hold on
            for i = 1:obj.num_robots
                x = obj.val(2*i-1:2*i);
                obj.head_handle = [obj.head_handle,plot(x(1),x(2),'Marker',head_shape{i},'MarkerSize',head_size,'MarkerEdgeColor',robot_color{i},'MarkerFaceColor',robot_color{i})];
                % write the text next to the robot
                if ~isempty(robot_text)
                    text_pos= x(1:2)+2; % we shift the text a little bit away from the node. % we add the mid-point of the base of this bigger robot to its heading position to find the text position, right behind the robot.
                    text_pos(1) = text_pos(1) - 0.45; % for some reason MATLAB shifts the starting point of the text a little bit to the right. So, here we return it back.
                    obj.text_handle = [obj.text_handle,text(text_pos(1),text_pos(2),robot_text,'fontsize',font_size,'color',text_color)];
                end
            end
            set(gca,'NextPlot',tmp);
        end
        function obj = delete_plot(obj,varargin)
            % The list of properties for this function is:
            % 'triangle', 'head'.
            if isempty(varargin)
                try % Avoid errors if the graphic object has already been deleted
                    delete(obj.head_handle);
                    obj.head_handle = [];
                end
                try % Avoid errors if the graphic object has already been deleted
                    delete(obj.tria_handle);
                    obj.tria_handle = [];
                end
                try % Avoid errors if the graphic object has already been deleted
                    delete(obj.text_handle);
                    obj.text_handle = [];
                end
            else
                for i = 1 : length(varargin)
                    switch varargin{i}
                        case 'triangle'
                            try % Avoid errors if the graphic object has already been deleted
                                delete(obj.tria_handle);
                                obj.tria_handle = [];
                            end
                        case 'head'
                            try % Avoid errors if the graphic object has already been deleted
                                delete(obj.head_handle)
                                obj.head_handle = [];
                            end
                        case 'text'
                            try % Avoid errors if the graphic object has already been deleted
                                delete(obj.text_handle);
                                obj.text_handle = [];
                            end
                        otherwise
                            error('There is no such a handle to delete')
                    end
                end
            end
        end
        function neighb_plot_handle = draw_neighborhood(obj, scale)
             tmp_th = 0:0.1:2*pi;
             neighb_plot_handle = [];
             for i = 1:obj.num_robots
                 x = obj.val(2*i-1);
                 y = obj.val(2*i);
                 tmp_h = plot(scale*cos(tmp_th) + x , scale*sin(tmp_th) + y);
                 neighb_plot_handle = [neighb_plot_handle , tmp_h]; %#ok<AGROW>
             end
        end
        function YesNo = is_constraint_violated(obj)
            YesNo = 0; % initialization
            obst = obstacles_class.obst; % for abbreviation
            for j = 1:obj.num_robots
                x = obj.val(2*j-1:2*j);
                for i_ob = 1:length(obst)
                    if any(inpolygon(x(1,:),x(2,:),obst{i_ob}(:,1),obst{i_ob}(:,2)))
                        YesNo =1;
                        return
                    end
                end
            end
        end
        function old_limits = zoom_in(obj,zoom_ratio) %#ok<INUSD>
            old_xlim = xlim;
            old_ylim = ylim;
            old_limits = [old_xlim,old_ylim];
%             new_center = obj.joint_2D_locations(:,end);
%             new_x_length = (old_xlim(2)-old_xlim(1))/zoom_ratio;
%             new_y_length = (old_ylim(2)-old_ylim(1))/zoom_ratio;
%             new_xlim = new_center(1) + [-new_x_length,new_x_length]/2;
%             new_ylim = new_center(2) + [-new_y_length,new_y_length]/2;
%             axis([new_xlim,new_ylim])
        end
    end
    
    methods (Static)
        function sampled_state = sample_a_valid_state()
            product_space_sampling_flag = 0;
            if product_space_sampling_flag == 1
                sampled_state = product_space_sampling();
                return
            end
            disp('we have to check the validity also')
            robots_val = [];
            for i = 1:state.num_robots
                [x,y]=ginput(1);
                if isempty(x)
                    sampled_state = [];
                    return
                else
                    robots_val = [robots_val ; [x ; y ] ]; %#ok<AGROW>
                end
            end
            sampled_state = state(robots_val);
        end
    end
end

function sampled_state_product_space = product_space_sampling()
if state.dim ~= 6, error('this function only works for 3 planar (2D) robots'); end
sampled_state = state.empty;
finish_flag = 0;
max_samples = 20;
for i_samp = 1:max_samples
    %---------we select a VALID single joint sample
    while 1
        [x1,y1]=ginput(1);
        if isempty(x1)
            finish_flag = 1;
            break
        end
        window_size = 5;
        x2 = x1+rand*window_size - window_size/2;
        x3 = x1+rand*window_size - window_size/2;
        y2 = y1+rand*window_size - window_size/2;
        y3 = y1+rand*window_size - window_size/2;
        tmp_state = state([x1;y1;x2;y2;x3;y3]);
        if ~tmp_state.is_constraint_violated()
            break
        end
    end
    %----------
    if finish_flag == 1
        break
    else
        tmp_state.draw();
        sampled_state = [sampled_state , tmp_state];
    end
end

%for i_rob = 1:state.num_robots
n_s = length(sampled_state);
sampled_state_product_space = state.empty(0,n_s^3);
for i_samp = 1:n_s
    for j_samp = 1:n_s
        for k_samp = 1:n_s
            product_sample = state ( [sampled_state(i_samp).val(1:2) ; sampled_state(j_samp).val(3:4) ; sampled_state(k_samp).val(5:6)] );
            sampled_state_product_space = [sampled_state_product_space , product_sample];
        end
    end
end
end










