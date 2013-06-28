classdef Planar_manipulator_wall_sensing < ObservationModel_interface
    % Note that because the class is defined as a handle class, the
    % properties must be defined such that they are do not change from an
    % object to another one.
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
%         tmp_prop = Planar_manipulator_wall_sensing.get_landmarks();  % This property is not needed!! I only added it because I could not find any other way to initialize the rest of constant properties. 
        L = [-1;1];
        obsDim = state.dim;
        obsNoiseDim = state.dim; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
        eta = 0.11;
        sigma_b = 0.2;
    end
    properties
%        landmarks_handle = Planar_manipulator_wall_sensing.tmp_prop.plot_handle;
    end
    
    methods (Static = true)
        function temporary_props = get_landmarks()
%             LoadFileName = user_data_class.par.LoadFileName;
%             SaveFileName = user_data_class.par.SaveFileName;
%             Man_L = user_data_class.par.observation_model_parameters.interactive_OM;
%             if Man_L == 0
%                 temporary_props = Planar_manipulator_wall_sensing.load_and_draw_landmarks(LoadFileName);
%             else
%                 temporary_props = Planar_manipulator_wall_sensing.request_landmarks();
%             end
%             Landmarks = temporary_props.landmarks; %#ok<NASGU>
%             save(SaveFileName,'Landmarks','-append') % here, we save the landmarks for the future runs.
        end
        function temporary_props = load_and_draw_landmarks(LoadFileName)
%             load(LoadFileName,'Landmarks')
%             temporary_props.landmarks = Landmarks;
%             temporary_props.plot_handle = Planar_manipulator_wall_sensing.draw(Landmarks);
        end
        function handle_of_plot = draw(landmarks)
%             old_prop = Planar_manipulator_wall_sensing.set_figure();
%             handle_of_plot = plot(landmarks(1,:),landmarks(2,:),'kp','markerfacecolor','k','markersize',12);
%             Planar_manipulator_wall_sensing.reset_figure(old_prop);
        end
        function temporary_props = request_landmarks()
%             old_prop = Planar_manipulator_wall_sensing.set_figure();
%             i=0;
%             title({'Please mark Landmarks'},'fontsize',14)
%             while true
%                 i=i+1;
%                 [Lx_temp,Ly_temp]=ginput(1);
%                 if isempty(Lx_temp) && i<3
%                     title({'You have to choose at least 3 landmarks to have an observable system'},'fontsize',14)
%                     i=i-1;
%                     continue
%                 elseif isempty(Lx_temp) && i>=3
%                     break
%                 else
%                     Lx(i)=Lx_temp; %#ok<AGROW>
%                     Ly(i)=Ly_temp; %#ok<AGROW>
%                     temporary_props.plot_handle(i)=plot(Lx(i),Ly(i),'kp','markerfacecolor','k','markersize',12);
%                 end
%             end
%             Landmarks=[Lx;Ly];
%             temporary_props.landmarks = Landmarks;
%             
%             N_L=size(Landmarks,2);
%             n = Planar_manipulator_wall_sensing.num_robots;
%             num_rob_in_group1 = floor(n/2); % number of robots in group 1
%             num_rob_in_group2 = n - num_rob_in_group1; % number of robots in group 2
%             N_L1 = floor(N_L/2); % number of landmarks that are visible for group 1 of robots
%             N_L2 = N_L - N_L1; % number of landmarks that are visible for group 2 of robots
%             num_obs_by_group1 = num_rob_in_group1*N_L1*2; % Number of all observation that are gained by group 1 of robots
%             num_obs_by_group2 = num_rob_in_group2*N_L2*2; % Number of all observation that are gained by group 2 of robots
%             total_num_obs = num_obs_by_group1 + num_obs_by_group2;
%             
%             temporary_props.obsDim = total_num_obs;
%             Planar_manipulator_wall_sensing.reset_figure(old_prop);
        end
        %% Sesing model
        % $$ z = h(x)=\left( \begin{array}{c} z_1 \\ z_2 \\ \vdots \\ z_N \end{array} \right) $$
        function z = h_func(x,v)
            z = x + v;
        end
        function H = dh_dx_func(x,v) %#ok<INUSD>
            H = eye(Planar_manipulator_wall_sensing.obsDim);
        end
        function M = dh_dv_func(x,v) %#ok<INUSD>
            % Jacobian of observation wrt observation noise.
            M = eye(Planar_manipulator_wall_sensing.obsDim);
        end
        function V = generate_observation_noise(x)
            obsDim = Planar_manipulator_wall_sensing.obsDim;%#ok<PROP>
            R = Planar_manipulator_wall_sensing.noise_covariance(x);
            indep_part_of_obs_noise=randn(obsDim,1); %#ok<PROP>
            V = indep_part_of_obs_noise.*diag(R.^(1/2));
        end
        function R = noise_covariance(x)
            x_state = state(x);
            std_vector = nan(Planar_manipulator_wall_sensing.obsDim , 1);
            for i = 1:Planar_manipulator_wall_sensing.obsDim
                std_vector(i) = Planar_manipulator_wall_sensing.eta * norm(x_state.joint_2D_locations(:,i) - Planar_manipulator_wall_sensing.L) + Planar_manipulator_wall_sensing.sigma_b;
            end
            R=diag(std_vector.^2);
        end
        function innov = compute_innovation(Xprd,Zg)
            V = zeros(Planar_manipulator_wall_sensing.obsNoiseDim,1);
            Zprd = Planar_manipulator_wall_sensing.h_func(Xprd,V);
            innov = Zg - Zprd;
            wrong_innovs = find(innov>pi | innov<-pi);
            for jjj=1:length(wrong_innovs)
                i=wrong_innovs(jjj);
                if mod(i,2)==0 && innov(i)>pi
                    innov(i)=innov(i)-2*pi;
                elseif mod(i,2)==0 && innov(i)<-pi
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
end

