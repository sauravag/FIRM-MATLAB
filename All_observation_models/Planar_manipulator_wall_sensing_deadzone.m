classdef Planar_manipulator_wall_sensing_deadzone < ObservationModel_interface
    % Note that because the class is defined as a handle class, the
    % properties must be defined such that they are do not change from an
    % object to another one.
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        tmp_prop = Planar_manipulator_wall_sensing_deadzone.costant_property_constructor();  % This property is not needed!! I only added it because I could not find any other way to initialize the rest of constant properties.
        L = [-1.5;0];
        obsDim = state.dim;
        obsNoiseDim = state.dim; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
        eta = 1;
        sigma_b = 10^(-7);
        thresh = 1;
        visible_region_std = 10^(-2);
    end
    properties
        plot_handle
    end
    
    methods (Static = true)
        function tmp_prop = costant_property_constructor()
            tmp_prop = [];
        end
        %% Sesing model
        % $$ z = h(x)=\left( \begin{array}{c} z_1 \\ z_2 \\ \vdots \\ z_N \end{array} \right) $$
        function z = h_func(x,v)
            z = x + v;
        end
        function H = dh_dx_func(x,v) %#ok<INUSD>
            H = eye(Planar_manipulator_wall_sensing_deadzone.obsDim);
        end
        function M = dh_dv_func(x,v) %#ok<INUSD>
            % Jacobian of observation wrt observation noise.
            M = eye(Planar_manipulator_wall_sensing_deadzone.obsDim);
        end
        function V = generate_observation_noise(x)
            obsDim = Planar_manipulator_wall_sensing_deadzone.obsDim;%#ok<PROP>
            R = Planar_manipulator_wall_sensing_deadzone.noise_covariance(x);
            indep_part_of_obs_noise=randn(obsDim,1); %#ok<PROP>
            V = indep_part_of_obs_noise.*diag(R.^(1/2));
        end
        function R = noise_covariance(x)
            x_state = state(x);
            std_vector = nan(Planar_manipulator_wall_sensing_deadzone.obsDim , 1);
            
            for i = 1:Planar_manipulator_wall_sensing_deadzone.obsDim
                norm_d = norm(x_state.joint_2D_locations(1,i) - Planar_manipulator_wall_sensing_deadzone.L(1));
                if norm_d < Planar_manipulator_wall_sensing_deadzone.thresh
                    std_vector(i) = Planar_manipulator_wall_sensing_deadzone.visible_region_std;
                else
                    std_vector(i) = Planar_manipulator_wall_sensing_deadzone.eta *norm_d  + Planar_manipulator_wall_sensing_deadzone.sigma_b;
                end
            end
            
            R=diag(std_vector.^2);
        end
        function innov = compute_innovation(Xprd,Zg)
            V = zeros(Planar_manipulator_wall_sensing_deadzone.obsNoiseDim,1);
            Zprd = Planar_manipulator_wall_sensing_deadzone.h_func(Xprd,V);
            innov = Zg - Zprd;
            wrong_innovs = find(innov>pi | innov<-pi);
            for jjj=1:length(wrong_innovs)
                i=wrong_innovs(jjj);
                if innov(i)>pi
                    innov(i)=innov(i)-2*pi;
                elseif innov(i)<-pi
                    innov(i)=innov(i)+2*pi;
                end
            end
        end
        function old_prop = set_figure() % This function sets the figure (size and other properties) to values that are needed for landmark selection or drawing.
            figure(gcf);
            old_prop{1}=get(gca,'NextPlot');hold on; % save the old "NextPlot" property and set it to "hold on" % Note that this procedure cannot be moved into the "set_figure" function.
            old_prop{2}=get(gca,'XGrid'); % save the old "XGrid" property.
            old_prop{3}=get(gca,'YGrid'); % save the old "YGrid" property.
            grid on; % set the XGrid and YGrid to "on".
            if ~isempty(user_data_class.par.figure_position)
                set(gcf,'Position',user_data_class.par.figure_position)
            end
            axis(user_data_class.par.env_limits);
            set(gca,'DataAspectRatio',[1 1 1]); % makes the scaling of different axes the same. So, a circle is shown as a circle not ellipse.
        end
        function reset_figure(old_prop) % This function resets the figure properties (size and other properties), to what they were before setting them in this class.
            set(gca,'NextPlot',old_prop{1}); % reset the "NextPlot" property to what it was.
            set(gca,'XGrid',old_prop{2}); % reset  the "XGrid" property to what it was.
            set(gca,'YGrid',old_prop{3}); % reset  the "YGrid" property to what it was.
        end
    end
    
    methods
        function obj = draw(obj)
            % this figure is almost meaningless, but it is something at
            % least to show
            old_prop = Planar_manipulator_wall_sensing_deadzone.set_figure();
            x_gird = user_data_class.par.env_limits(1):0.1:user_data_class.par.env_limits(2);
            y_gird = user_data_class.par.env_limits(3):0.1:user_data_class.par.env_limits(4);
            [x_mesh,y_mesh] = meshgrid(x_gird , y_gird);
            eta_local = Planar_manipulator_wall_sensing_deadzone.eta;
            L_local = Planar_manipulator_wall_sensing_deadzone.L(1);
            sigma_b_local = Planar_manipulator_wall_sensing_deadzone.sigma_b;
            z_mesh = zeros(size(x_mesh,1),size(x_mesh,2));
            for i = 1:size(x_mesh,1)
                for j = 1:size(x_mesh,2)
                    norm_d = norm([x_mesh(i,j)] - L_local);
                    if norm_d < 0.1;%Planar_manipulator_wall_sensing_deadzone.thresh
                        z_mesh(i,j) = (Planar_manipulator_wall_sensing_deadzone.visible_region_std)^2;
                    else
                        z_mesh(i,j) = (eta_local * norm_d + sigma_b_local)^2;
                    end
                end
            end
            obj.plot_handle = surf(x_mesh,y_mesh,-z_mesh);
            caxis([-9 0])
            cmap = repmat(linspace(0.2,1,64)',1,3);
            colormap(cmap);
%             handle_of_plot = [handle_of_plot , plot(Planar_manipulator_wall_sensing_deadzone.L(1,:),Planar_manipulator_wall_sensing_deadzone.L(2,:),'kp','markerfacecolor','k','markersize',12)];
            shading interp
            Planar_manipulator_wall_sensing_deadzone.reset_figure(old_prop);
        end
        function obj = delete_plot(obj)
            delete(obj.plot_handle)
            obj.plot_handle = [];
        end
    end
end

