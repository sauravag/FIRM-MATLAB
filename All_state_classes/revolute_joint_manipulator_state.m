classdef revolute_joint_manipulator_state < state_interface
    % This class encapsulates the state of the system.
    properties (Constant)
        num_revolute_joints = user_data_class.par.state_parameters.num_revolute_joints;
        dim = revolute_joint_manipulator_state.num_revolute_joints; % state dimension
        link_length = 0.5;
    end
    properties
        val; % value of the state
        joint_2D_locations; % 2D locations of joints in the plane
        plot_handle; % handle for the "drawings" associated with the state
        text_handle; % handle for the "displayed text" associated with the state
    end
    
    
    methods
        function obj = revolute_joint_manipulator_state(varargin) % constructor function
            obj = obj@state_interface(varargin{:});
            th = obj.val;
            sum_theta = 0; % initialization % since teh manipulator works in a plane, the rotation has to be a 2by2 matrix
            end_link = [0;0]; % this is the location in the plane that the beginning of the manipulator is attached to
            for i = 1:revolute_joint_manipulator_state.num_revolute_joints
                sum_theta = sum_theta + th(i);  % at i-th iteration R_prod is the product of R1*R2*...*Ri
                start_link = end_link(:,max(i-1,1)); % 2D position of the start of i-th link in plane
                end_link(:,i) = obj.Rot_2D(sum_theta)*[obj.link_length;0] + start_link; % 2D position of the end of i-th link in plane  % [obj.link_length;0] means that the length of the i-th joint is 1.
            end
            obj.joint_2D_locations = end_link; % Note that the i-th column of this matrix is the 2D location of the end of i-th link in the manipulator.
        end
    end
    
    methods
        function signed_dist_vector = signed_element_wise_dist(obj,x2) % this function returns the "Signed element-wise distnace" between two states x1 and x2
            x1 = obj.val; % retrieve the value of the state vector
            if isa(x2,'revolute_joint_manipulator_state'), x2=x2.val; end % retrieve the value of the state vector
            signed_dist_vector = x1 - x2;
            % Following part takes care of periodicity in heading angle representation.
            for i = 1:obj.num_revolute_joints
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
        function obj = draw(obj, varargin) % draw state
            manip_color = 'b'; %default color
            robot_text = [];
            font_size = 15;
            text_color  = 'b';
            for i = 1 : 2 : length(varargin)
                switch lower(varargin{i})
                    case lower('color')
                        manip_color = varargin{i+1};
                    case lower('text')
                        robot_text = varargin{i+1};
                    case lower('fontsize')
                        font_size = varargin{i+1};
                    case lower('textcolor')
                        text_color = varargin{i+1};
                end
            end
            all_joints = [ [0;0] , obj.joint_2D_locations ];
            obj.plot_handle = plot( all_joints(1,:) , all_joints(2,:) , 'color', manip_color);
            if ~isempty(robot_text)
                text_pos = obj.joint_2D_locations(:,end);
                text_pos(1) = text_pos(1) - 0.45; % for some reason MATLAB shifts the starting point of the text a little bit to the right. So, here we return it back.
                obj.text_handle = text(text_pos(1),text_pos(2),robot_text,'fontsize',font_size,'color',text_color);
            end
        end
        function obj = delete_plot(obj,varargin) % delete state drawings
            try
            delete(obj.plot_handle);
            end
            obj.plot_handle =[];
        end
        function neighb_plot_handle = draw_neighborhood(obj, scale)
            disp('Not yet implemented');
            neighb_plot_handle = 'something';
        end
        function YesNo = is_constraint_violated(obj) % this function checks if the "state" is a collilsion-free one or not.
            YesNo = 0; % initialization
            all_joints = [ [0;0] , obj.joint_2D_locations ]; % Actually, this is a polyline 
            % first we check the self-intersections
%             [x_inters,~] = polyxpoly(all_joints(1,:),all_joints(2,:),all_joints(1,:),all_joints(2,:)); % the intersection points must be the vertices of the manipulator polygon
%             if length(x_inters)>size(all_joints,2) % if there is any more intersection points than the number of manipulator joints, that means that we have a self-intersection
%                 YesNo=1;
%                 return
%             end
            % now, we check the intersection with obstacls
            Obst = obstacles_class.obst;
            N_obst=size(Obst,2);
            for ib=1:N_obst
                X_obs=[Obst{ib}(:,1);Obst{ib}(1,1)];
                Y_obs=[Obst{ib}(:,2);Obst{ib}(1,2)];
                [x_inters,~] = polyxpoly(X_obs,Y_obs,all_joints(1,:),all_joints(2,:));
                if ~isempty(x_inters)
                    YesNo=1;
                    return
                end
            end
        end
        function old_limits = zoom_in(obj,zoom_ratio)
            old_xlim = xlim;
            old_ylim = ylim;
            old_limits = [old_xlim,old_ylim];
            new_center = obj.joint_2D_locations(:,end);
            new_x_length = (old_xlim(2)-old_xlim(1))/zoom_ratio;
            new_y_length = (old_ylim(2)-old_ylim(1))/zoom_ratio;
            new_xlim = new_center(1) + [-new_x_length,new_x_length]/2;
            new_ylim = new_center(2) + [-new_y_length,new_y_length]/2;
            axis([new_xlim,new_ylim])
        end
    end
    
    methods (Static)
        function sampled_state = sample_a_valid_state()
            user_or_random = 'random';
            if strcmp(user_or_random , 'user')
                sampled_state = user_samples_a_state();
            elseif strcmp(user_or_random , 'random')
                sampled_state = randomly_sample_a_state();
            else
                error('not correct!');
            end
        end
    end
    
    methods (Access = private)
        
            
        function draw_link()
        end
        function R = Rot_2D(obj,th)
            R = [cos(th) -sin(th);sin(th) cos(th)];
        end
        end
        
end

function sampled_state = user_samples_a_state()
x_circle = cos(0:0.1:2*pi)*revolute_joint_manipulator_state.link_length; y_circle = sin(0:0.1:2*pi)*revolute_joint_manipulator_state.link_length; % to plot indicator (helper) circles
end_link = [0;0]; % this is the location in the plane that the beginning of the manipulator is attached to
for i = 1:revolute_joint_manipulator_state.num_revolute_joints
    circle_handle = plot(x_circle + end_link(1), y_circle + end_link(2));
    circle_handle = [circle_handle , plot(end_link(1),end_link(2),'*')];
    [x,y]=ginput(1);
    if isempty(x)
        delete(circle_handle);
        sampled_state = [];
        return
    else
        start_link = end_link; % 2D position of the start of i-th link in plane
        rel_vector = [x;y] - start_link; % relative vector
        rel_vector = rel_vector/norm(rel_vector)*revolute_joint_manipulator_state.link_length; % making the length of link equal to "state.link_length"
        end_link = rel_vector + start_link;
        tmp_h = plot( [start_link(1) , end_link(1)] , [start_link(2) , end_link(2)] ,'r');
        delete(circle_handle);
        th_absolute(i) = atan2(end_link(2) - start_link(2) , end_link(1) - start_link(1)) ;
    end
end
th_joints = [th_absolute(1) , diff(th_absolute)];
for i = 1:revolute_joint_manipulator_state.num_revolute_joints
    th_dist = th_joints(i);
    if th_dist >= 0
        th_dist_bounded = mod( th_dist , 2*pi );
        if th_dist_bounded > pi, th_dist_bounded = th_dist_bounded - 2*pi; end
    else
        th_dist_bounded = - mod( -th_dist , 2*pi );
        if th_dist_bounded < -pi, th_dist_bounded = th_dist_bounded + 2*pi; end
    end
    th_joints(i) = th_dist_bounded;   % updating the angle distance
end
sampled_state = revolute_joint_manipulator_state(th_joints);
if sampled_state.is_constraint_violated()
    sampled_state = user_samples_a_state();
end
end
function sampled_state = randomly_sample_a_state()
sampled_state_val = rand(revolute_joint_manipulator_state.dim,1)*2*pi-pi;
sampled_state = revolute_joint_manipulator_state(sampled_state_val);
if sampled_state.is_constraint_violated()
    sampled_state = randomly_sample_a_state();
end
end
