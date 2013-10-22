classdef Six_DOF_robot_state < state_interface
    % This class encapsulates the state of a planar robot, described by its 2D location and heading angle.
    properties (Constant)
        dim = 7; % state dimension [X,Y,Z,q0,q1,q2,q3]
        zeroQuaternion = [1 0 0 0];
    end
    properties
        val; % value of the state
        plot_handle; % handle for the "drawings" associated with the state
        head_handle;
        tria_handle;
        text_handle; % handle for the "displayed text" associated with the state
    end
    
    methods
        function obj = Six_DOF_robot_state(varargin)
            obj = obj@state_interface(varargin{:});
        end
        function signed_dist_vector = signed_element_wise_dist(obj,x2) % this function returns the "Signed element-wise distance" between two states x1 and x2
             x1 = obj.val; % retrieve the value of the state vector
             if isa(x2,'state'), x2=x2.val; end % retrieve the value of the state vector
             linear_distance = x1(1:3,1) - x2(1:3,1) ; % [X1-X2, Y1-Y2, Z1-Z2]'
             %The relative quaternion so to speak 
             % q_rel = q_current * inv(q_nominal)
             q_1 = [x1(4) x1(5) x1(6) x1(7)];
             q_2 = [x2(4) x2(5) x2(6) x2(7)];
             q21 = quatmultiply(q_1,quatinv(q_2)); % relative rotation quaternion from nominal to current
             signed_dist_vector = [linear_distance;q21'];
        end
        function obj = draw(obj, varargin)
            % The full list of properties for this function is:
            % 'RobotShape', 'RobotSize', 'TriaColor', 'color', 'HeadShape',
            % 'HeadSize'.
            
            % default values
            robot_shape = 'point';
            robot_size = 0.1;
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
            obj.head_handle = plot3(x(1),x(2),x(3),'Marker',head_shape,'MarkerSize',head_size,'MarkerEdgeColor',head_color,'MarkerFaceColor',head_color);
            robot_size = 0.4;
%             vertices_x=[0,-robot_size,-robot_size,0];
%             vertices_y=[0,-robot_size/5,robot_size/5,0];
%             vertices_z=[0,0,0,0];
            vertices_x=[0,-robot_size,-robot_size,-robot_size,-robot_size/2,-robot_size,-robot_size,0];
            vertices_y=[0,-robot_size/5,0,0,0,0,robot_size/5,0];
            vertices_z=[0,0,0,robot_size/10,0,0,0,0];
            
            q_rot = Quaternion(x(4:7)); % rotation quaternion
            q_rot = unit(q_rot);% making a normalized quarternion, dont use quatnormalize
            R = q_rot.R;
            vert=R*[vertices_x;vertices_y;vertices_z];
            % plot the triangle body
            if strcmpi(robot_shape,'triangle')  % strcmpi is not sensitive to letter scase.
                tmp=get(gca,'NextPlot');
                hold on
                obj.tria_handle = plot3(vert(1,:)+x(1),vert(2,:)+x(2),vert(3,:)+x(3),'color',tria_color);
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
            
%             obj.tria_handle = [obj.tria_handle,plot_3D_cone_in_2D(x,tria_color)];
            
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
            x = obj.val(1);
            y = obj.val(2);
            neighb_plot_handle = plot3(scale*cos(tmp_th) + x , scale*sin(tmp_th) + y, obj.val(3));
        end
        function YesNo = is_constraint_violated(obj)
            x = obj.val;
            YesNo = 0; % initialization
            obst = obstacles_class.obst; % for abbreviation
            for i_ob = 1:length(obst)
                if any(inpolygon(x(1,:),x(2,:),obst{i_ob}(:,1),obst{i_ob}(:,2)))
                    YesNo =1;
                    return
                end
            end
        end
        function old_limits = zoom_in(obj,zoom_ratio)
            old_xlim = xlim;
            old_ylim = ylim;
            old_limits = [old_xlim,old_ylim];
            new_center = obj.val;
            new_x_length = (old_xlim(2)-old_xlim(1))/zoom_ratio;
            new_y_length = (old_ylim(2)-old_ylim(1))/zoom_ratio;
            new_xlim = new_center(1) + [-new_x_length,new_x_length]/2;
            new_ylim = new_center(2) + [-new_y_length,new_y_length]/2;
            axis([new_xlim,new_ylim])
        end
    end
    
    methods (Static)
        function sampled_state = sample_a_valid_state()
            user_or_random = 'user';
            xrange = [0.5 4];
            yrange = [0.5 4];
            zrange = [0.5 4];
            if strcmp(user_or_random , 'user')
                [x,y]=ginput(1);
                if isempty(x)
                    sampled_state = [];
                    return
                else
                    xyz = rand(1,3) .* [xrange(2)-xrange(1) yrange(2)-yrange(1) zrange(2)-zrange(1)] + ...
                        [xrange(1) yrange(1) zrange(1)];
                    
                    yaw = rand*2*pi;
                    pitch = -pi/2 + rand*pi;
                    roll =  -pi/2 + rand*pi;
                    quat = angle2quat(yaw,pitch,roll);
                    sampled_state = state([x , y , xyz(3), quat]');
                end
                if sampled_state.is_constraint_violated()
                    sampled_state = user_samples_a_state();
                end
            elseif strcmp(user_or_random , 'random')
                xyz = rand(1,3) .* [xrange(2)-xrange(1) yrange(2)-yrange(1) zrange(2)-zrange(1)] + ...
                    [xrange(1) yrange(1) zrange(1)];
                
                yaw = rand*2*pi;
                pitch = rand*2*pi;
                roll =  rand*2*pi;
                quat = angle2quat(yaw,pitch,roll);
                sampled_state = [xyz, quat]';
            else
                error('not correct!');
            end
        end
    end
end

function plot_handle = plot_3D_cone_in_2D(x,color)

size_ratio = 0.75;
r = 1*size_ratio;
h = 2*size_ratio;
m = h/r;
[R,A] = meshgrid(linspace(0,r,2),linspace(0,2*pi,20));
X = R .* cos(A);
Y = R .* sin(A);
Z = m*R;
X_base = X;
Y_base = Y;
Z_base = h*ones(size(X_base));
% Cone around the z-axis, point at the origin
% mesh(X,Y,Z)
X = [X,X_base];
Y = [Y,Y_base];
Z = [Z,Z_base];

% mesh(X,Y,Z)

% axis equal
% axis([-3 3 -3 3 0 3])

phi = pi/2;
X1 = X*cos(phi) - Z*sin(phi);
Y1 = Y;
Z1 = X*sin(phi) + Z*cos(phi);
% Previous cone, rotated by angle phi about the y-axis
%mesh(X1,Y1,Z1)
% h_cone = surf(X1,Y1,Z1);
% set(h_cone,'edgecolor','none','facecolor',color,'facelighting','gouraud');
rot  = [x(4) ; x(5) ; x(6) ; x(7)];% rotation state
q_rot = Quaternion(rot);
q_rot = unit(q_rot);% making a normalized quarternion, dont use quatnormalize
Pos = q_rot.R*[X1;Y1;Z1] + repmat(x(1:3),1,size(X1));
X2 = Pos(1,:);
Y2 = Pos(2,:);
Z2 = Pos(3,:);
% Second cone rotated by angle theta about the z-axis
plot_handle = mesh(X2,Y2,Z2);
% axis equal
% set(gca,'PlotBoxAspectRatio',[1 1 1]);
% camlight(135,180);
% view([30,25]);
set(plot_handle,'edgecolor','none','facecolor',color,'facelighting','phong');


% xlabel('x')
% ylabel('y')
% zlabel('z')

end