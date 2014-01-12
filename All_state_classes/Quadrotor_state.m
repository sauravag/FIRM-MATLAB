classdef Quadrotor_state < state_interface
    % This class encapsulates the 12 dimensional state space of a quadrotor
    properties (Constant)
        dim = 12; % state dimension
    end
    properties
        val; % value of the state
        plot_handle; % handle for the "drawings" associated with the state
        text_handle; % handle for the "displayed text" associated with the state
    end
    
    methods
        function obj = Quadrotor_state(varargin)
            obj = obj@state_interface(varargin{:});
        end
        function signed_dist_vector = signed_element_wise_dist(obj,x2) % this function returns the "Signed element-wise distance" between two states x1 and x2
             x1 = obj.val; % retrieve the value of the state vector
             if isa(x2,'state'), x2=x2.val; end % retrieve the value of the state vector
             signed_dist_vector = x1 - x2;
            % Following part takes care of periodicity in heading angle representation.
            for i = 4:6
                th_dist = signed_dist_vector(i);
                if th_dist >= 0
                    th_dist_bounded = mod( th_dist , 2*pi );
                    if th_dist_bounded > pi, th_dist_bounded = th_dist_bounded - 2*pi; end
                else
                    th_dist_bounded = - mod( -th_dist , 2*pi );
                    if th_dist_bounded < -pi, th_dist_bounded = th_dist_bounded + 2*pi; end
                end
                signed_dist_vector(i) = th_dist_bounded;   % updating the angle distance
            end
        end
        
       function obj = draw(obj, varargin)
            % The full list of properties for this function is:
            % 'RobotShape', 'RobotSize', 'TriaColor', 'color', 'HeadShape',
            % 'HeadSize'.
            
            % default values
            robot_shape = 'point';
            robot_size = 1;
            tria_color = 'b';
            head_color = 'b';
            head_shape = '*';
            head_size = 6;
            robot_text = [];
            font_size = 15;
            text_color  = 'b';
            for i = 1 : 2 : length(varargin)
                switch lower(varargin{i})
                    case lower('RobotShape')
                        robot_shape = varargin{i+1};
                    case lower('RobotSize')
                        robot_size = varargin{i+1};
                    case lower('TriaColor')
                        tria_color = varargin{i+1};
                    case lower('color')
                        head_color = varargin{i+1};
                    case lower('HeadShape')
                        head_shape = varargin{i+1};
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
            x=obj.val;
            obj.plot_handle.head_handle = plot3(x(1),x(2),x(3),'Marker',head_shape,'MarkerSize',head_size,'MarkerEdgeColor',head_color,'MarkerFaceColor',head_color);
            robot_size = 4;
            vertices_x=[0,-robot_size,-robot_size,-robot_size,-robot_size/2,-robot_size,-robot_size,0];
            vertices_y=[0,-robot_size/5,0,0,0,0,robot_size/5,0];
            vertices_z=[0,0,0,robot_size/10,0,0,0,0];
            
            R = eul2r(obj.val(4:6)');  % The "eul2r" is a function Peter Corke's Robotic Toolbox. The function accepts "row vector".
            vert=R*[vertices_x;vertices_y;vertices_z];
            % plot the triangle body
            if strcmpi(robot_shape,'triangle')  % strcmpi is not sensitive to letter scase.
                tmp=get(gca,'NextPlot');
                hold on
                obj.plot_handle.tria_handle = plot3(vert(1,:)+x(1),vert(2,:)+x(2),vert(3,:)+x(3),'color',tria_color);
                set(gca,'NextPlot',tmp);
            end
            % write the text next to the robot
%             if ~isempty(robot_text)
%                 robot_size_for_text=robot_size*1.5; % we need a slightly bigger robot so that the text does not interfere with actual robot.
%                 vertices_x=[0,-robot_size_for_text,-robot_size_for_text,0];
%                 vertices_y=[0,-robot_size_for_text/5,robot_size_for_text/5,0];
%                 R=[cos(th),-sin(th);sin(th),cos(th)];
%                 vert=R*[vertices_x;vertices_y];
%                 
%                 text_pos=(vert(:,2)+vert(:,3))/2  + x(1:2); % we add the mid-point of the base of this bigger robot to its heading position to find the text position, right behind the robot.
%                 text_pos(1) = text_pos(1) - 0.45; % for some reason MATLAB shifts the starting point of the text a little bit to the right. So, here we return it back.
%                 obj.text_handle = text(text_pos(1),text_pos(2),robot_text,'fontsize',font_size,'color',text_color);
%             end
            
%             obj.plot_handle.tria_handle = [obj.plot_handle.tria_handle,plot_3D_cone_in_2D(x,tria_color)];
            
        end
        function obj = delete_plot(obj,varargin)
            % The list of properties for this function is:
            % 'triangle', 'head'.
            if isempty(varargin)
                try % Avoid errors if the graphic object has already been deleted
                    delete(obj.plot_handle.head_handle);
                    obj.plot_handle.head_handle = [];
                end
                try % Avoid errors if the graphic object has already been deleted
                    delete(obj.plot_handle.tria_handle);
                    obj.plot_handle.tria_handle = [];
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
                                delete(obj.plot_handle.tria_handle);
                                obj.plot_handle.tria_handle = [];
                            end
                        case 'head'
                            try % Avoid errors if the graphic object has already been deleted
                                delete(obj.plot_handle.head_handle)
                                obj.plot_handle.head_handle = [];
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
            x = obj.val(1);
            y = obj.val(2);
            neighb_plot_handle = plot3(scale*cos(tmp_th) + x , scale*sin(tmp_th) + y, obj.val(3)*ones(size(tmp_th)));
        end
        function YesNo = is_constraint_violated(obj)
            x = obj.val;
            YesNo = 0; % initialization
            obst = obstacles_class.obst; % for abbreviation
            for i_ob = 1:length(obst)
                if any(inpolygon(x(1,:),x(2,:),obst{i_ob}(:,1),obst{i_ob}(:,2))) % This function just works for 2D obstacles. It should be updated to its #D version ASAP.
                    YesNo =1;
                    return
                end
            end
        end
        function old_limits = zoom_in(obj,zoom_ratio)
            old_limits = axis;
%             old_xlim = xlim;
%             old_ylim = ylim;
%             old_limits = [old_xlim,old_ylim];
%             new_center = obj.val;
%             new_x_length = (old_xlim(2)-old_xlim(1))/zoom_ratio;
%             new_y_length = (old_ylim(2)-old_ylim(1))/zoom_ratio;
%             new_xlim = new_center(1) + [-new_x_length,new_x_length]/2;
%             new_ylim = new_center(2) + [-new_y_length,new_y_length]/2;
%             axis([new_xlim,new_ylim])
        end
        function obj = apply_differentiable_constraints(obj)
            error('not implemented yet')
            qnorm = norm(obj.val(4:7));
            if qnorm ~=0
                obj.val(4:7) = obj.val(4:7)/qnorm;
            end
        end
        function J = get_differentiable_constraints_jacobian(obj)
            error('not implemented yet')
            q = obj.val(4:7);
            nq = norm(q);
            Jq = -nq^(-3)*(q*q')+nq^(-1)*eye(4);
            J = blkdiag(eye(3),Jq);
        end
    end
    
    methods (Static)
        function sampled_state = sample_a_valid_state()
            user_or_random = 'user';
            if strcmp(user_or_random , 'user')
                [x,y]=ginput(1);
                if isempty(x)
                    sampled_state = [];
                    return
                else
                    zrange = user_data_class.par.sim.env_z_limits;
                    random_z = rand*(zrange(2) - zrange(1))+zrange(1); % generates a random altitude
                    sampled_state = state([x ; y ; random_z ; zeros(9,1)]); % we set velocities and angular directions to zero at sampled states
                end
                if sampled_state.is_constraint_violated()
                    title('The selected sample is not valid. Select another one')
                    sampled_state = state.sample_a_valid_state();
                    title('Select the next sample')
                end
            elseif strcmp(user_or_random , 'random')
                error('not yet implemented')
            else
                error('not correct!');
            end
        end
    end
end
