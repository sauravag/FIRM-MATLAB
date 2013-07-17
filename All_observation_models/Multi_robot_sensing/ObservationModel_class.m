classdef ObservationModel_class < ObservationModel_interface
    % Note that because the class is defined as a handle class, the
    % properties must be defined such that they are do not change from an
    % object to another one.
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
        num_robots  = user_data_class.par.state_parameters.num_robots;
        tmp_prop = ObservationModel_class.get_landmarks();  % This property is not needed!! I only added it because I could not find any other way to initialize the rest of constant properties. 
        landmarks = ObservationModel_class.tmp_prop.landmarks;
        obsDim = ObservationModel_class.tmp_prop.obsDim;
        obsNoiseDim = ObservationModel_class.obsDim; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
        eta = user_data_class.par.eta;
        sigma_b = user_data_class.par.sigma_b;
    end
    properties
       landmarks_handle = ObservationModel_class.tmp_prop.plot_handle;
    end
    
    methods (Static = true)
        function temporary_props = get_landmarks()
            LoadFileName = user_data_class.par.LoadFileName;
            SaveFileName = user_data_class.par.SaveFileName;
            Man_L = user_data_class.par.observation_model_parameters.interactive_OM;
            if Man_L == 0
                temporary_props = ObservationModel_class.load_and_draw_landmarks(LoadFileName);
            else
                temporary_props = ObservationModel_class.request_landmarks();
            end
            Landmarks = temporary_props.landmarks; %#ok<NASGU>
            save(SaveFileName,'Landmarks','-append') % here, we save the landmarks for the future runs.
        end
        function temporary_props = load_and_draw_landmarks(LoadFileName)
            load(LoadFileName,'Landmarks')
            temporary_props.landmarks = Landmarks;
            
            N_L=size(Landmarks,2);
            n = ObservationModel_class.num_robots;
            num_rob_in_group1 = floor(n/2); % number of robots in group 1
            num_rob_in_group2 = n - num_rob_in_group1; % number of robots in group 2
            N_L1 = floor(N_L/2); % number of landmarks that are visible for group 1 of robots
            N_L2 = N_L - N_L1; % number of landmarks that are visible for group 2 of robots
            num_obs_by_group1 = num_rob_in_group1*N_L1*2; % Number of all observation that are gained by group 1 of robots
            num_obs_by_group2 = num_rob_in_group2*N_L2*2; % Number of all observation that are gained by group 2 of robots
            total_num_obs = num_obs_by_group1 + num_obs_by_group2;
            
            temporary_props.obsDim = total_num_obs;
            temporary_props.plot_handle = ObservationModel_class.draw(Landmarks);
        end
        function handle_of_plot = draw(landmarks)
            old_prop = ObservationModel_class.set_figure();
            handle_of_plot = plot(landmarks(1,:),landmarks(2,:),'kp','markerfacecolor','k','markersize',12);
            ObservationModel_class.reset_figure(old_prop);
        end
        function temporary_props = request_landmarks()
            old_prop = ObservationModel_class.set_figure();
            i=0;
            title({'Please mark Landmarks'},'fontsize',14)
            while true
                i=i+1;
                [Lx_temp,Ly_temp]=ginput(1);
                if isempty(Lx_temp) && i<3
                    title({'You have to choose at least 3 landmarks to have an observable system'},'fontsize',14)
                    i=i-1;
                    continue
                elseif isempty(Lx_temp) && i>=3
                    break
                else
                    Lx(i)=Lx_temp; %#ok<AGROW>
                    Ly(i)=Ly_temp; %#ok<AGROW>
                    temporary_props.plot_handle(i)=plot(Lx(i),Ly(i),'kp','markerfacecolor','k','markersize',12);
                end
            end
            Landmarks=[Lx;Ly];
            temporary_props.landmarks = Landmarks;
            
            N_L=size(Landmarks,2);
            n = ObservationModel_class.num_robots;
            num_rob_in_group1 = floor(n/2); % number of robots in group 1
            num_rob_in_group2 = n - num_rob_in_group1; % number of robots in group 2
            N_L1 = floor(N_L/2); % number of landmarks that are visible for group 1 of robots
            N_L2 = N_L - N_L1; % number of landmarks that are visible for group 2 of robots
            num_obs_by_group1 = num_rob_in_group1*N_L1*2; % Number of all observation that are gained by group 1 of robots
            num_obs_by_group2 = num_rob_in_group2*N_L2*2; % Number of all observation that are gained by group 2 of robots
            total_num_obs = num_obs_by_group1 + num_obs_by_group2;
            
            temporary_props.obsDim = total_num_obs;
            ObservationModel_class.reset_figure(old_prop);
        end
        function z_team = h_func(x_team,v_team)
            L=ObservationModel_class.landmarks;
            od = ObservationModel_class.obsDim;
            N_L=size(L,2);
            n = ObservationModel_class.num_robots;
            
            num_rob_in_group1 = floor(n/2); % number of robots in group 1
            num_rob_in_group2 = n - num_rob_in_group1; % number of robots in group 2
            N_L1 = floor(N_L/2); % number of landmarks that are visible for group 1 of robots
            N_L2 = N_L - N_L1; % number of landmarks that are visible for group 2 of robots
            L_group1 = L(:,1:N_L1);  % landmarks that are visible for group 1 of robots
            L_group2 = L(:,N_L1+1:N_L);  % landmarks that are visible for group 2 of robots
            
            num_obs_by_group1 = num_rob_in_group1*N_L1*2; % Number of all observation that are gained by group 1 of robots
            num_obs_by_group2 = num_rob_in_group2*N_L2*2; % Number of all observation that are gained by group 2 of robots
            
            % observations made by the first group of robots
            z_team = [];
            for i = 1:num_rob_in_group1 % ie for the first half of robots
                x = x_team(3*i-2:3*i);
                d = L_group1-repmat(x(1:2),1,N_L1);
                z_single_robot(1:2:2*N_L1-1,1) = sqrt(d(1,:).^2+d(2,:).^2)';
                z_single_robot(2:2:2*N_L1  ,1) = atan2(d(2,:),d(1,:))'-x(3)';
                z_team = [z_team ; z_single_robot]; %#ok<AGROW>
            end
            z_team = z_team + v_team(1:num_obs_by_group1 , 1);
            % observations made by the second group of robots
            for i = num_rob_in_group1+1:n % ie for the first half of robots
                x = x_team(3*i-2:3*i);
                d = L_group2-repmat(x(1:2),1,N_L2);
                z_single_robot(1:2:2*N_L2-1,1) = sqrt(d(1,:).^2+d(2,:).^2)';
                z_single_robot(2:2:2*N_L2  ,1) = atan2(d(2,:),d(1,:))'-x(3)';
                z_team = [z_team ; z_single_robot]; %#ok<AGROW>
            end
            z_team(num_obs_by_group1+1:od) = z_team(num_obs_by_group1+1:od) + v_team(num_obs_by_group1+1:od , 1);
            
        end
        function H_team = dh_dx_func(x_team,v_team) %#ok<INUSD>
            L=ObservationModel_class.landmarks;
            od = ObservationModel_class.obsDim;
            N_L = size(L,2);
            n = ObservationModel_class.num_robots;
            
            num_rob_in_group1 = floor(n/2); % number of robots in group 1
            num_rob_in_group2 = n - num_rob_in_group1; % number of robots in group 2
            N_L1 = floor(N_L/2); % number of landmarks that are visible for group 1 of robots
            N_L2 = N_L - N_L1; % number of landmarks that are visible for group 2 of robots
            L_group1 = L(:,1:N_L1);  % landmarks that are visible for group 1 of robots
            L_group2 = L(:,N_L1+1:N_L);  % landmarks that are visible for group 2 of robots
            
            H_group1 = [];
            for i = 1:num_rob_in_group1
                x = x_team(3*i-2:3*i);
                for j=1:size(L_group1,2)
                    dj=x(1:2)-L_group1(:,j);
                    phi_j = atan2(dj(2),dj(1));
                    rj = sqrt(dj'*dj);
                    
                    H_single_robot(2*j-1:2*j,:) = [ cos(phi_j)  ,      sin(phi_j)  ,      0
                        -sin(phi_j)/rj,    cos(phi_j)/rj,     -1 ];
                    %                 H_debug=[dj(1)/rj        dj(2)/rj        0
                    %                     -dj(2)/(rj^2)   dj(1)/(rj^2)    -1];
                    %                 if any(any(H(2*j-1:2*j,:)-H_debug>1.0e-12))
                    %                     error('Error in dh_dx')
                    %                 end
                end
                H_group1 = [H_group1 , H_single_robot];
            end
            
            H_group2 = [];
            for i = num_rob_in_group1+1:n
                x = x_team(3*i-2:3*i);
                for j=1:size(L_group2,2)
                    dj=x(1:2)-L_group2(:,j);
                    phi_j = atan2(dj(2),dj(1));
                    rj = sqrt(dj'*dj);
                    
                    H_single_robot(2*j-1:2*j,:) = [ cos(phi_j)  ,      sin(phi_j)  ,      0
                                                                                -sin(phi_j)/rj,    cos(phi_j)/rj,     -1 ];
                    %                 H_debug=[dj(1)/rj        dj(2)/rj        0
                    %                     -dj(2)/(rj^2)   dj(1)/(rj^2)    -1];
                    %                 if any(any(H(2*j-1:2*j,:)-H_debug>1.0e-12))
                    %                     error('Error in dh_dx')
                    %                 end
                end
                H_group2 = [H_group2 , H_single_robot];
            end
            
            H_team = blkdiag(H_group1 , H_group2);
            
        end
        function M = dh_dv_func(x,v) %#ok<INUSD>
            % Jacobian of observation wrt observation noise.
            M = eye(ObservationModel_class.obsDim);
        end
        function V = generate_observation_noise(x)
            obsDim = ObservationModel_class.obsDim;%#ok<PROP>
            R = ObservationModel_class.noise_covariance(x);
            indep_part_of_obs_noise=randn(obsDim,1); %#ok<PROP>
            V = indep_part_of_obs_noise.*diag(R.^(1/2));
        end
        function R = noise_covariance(x_team)
            L=ObservationModel_class.landmarks;
            od = ObservationModel_class.obsDim;
            N_L=size(L,2);
            n = ObservationModel_class.num_robots;
            
            num_rob_in_group1 = floor(n/2); % number of robots in group 1
            num_rob_in_group2 = n - num_rob_in_group1; % number of robots in group 2
            N_L1 = floor(N_L/2); % number of landmarks that are visible for group 1 of robots
            N_L2 = N_L - N_L1; % number of landmarks that are visible for group 2 of robots
            L_group1 = L(:,1:N_L1);  % landmarks that are visible for group 1 of robots
            L_group2 = L(:,N_L1+1:N_L);  % landmarks that are visible for group 2 of robots
            
            num_obs_by_group1 = num_rob_in_group1*N_L1*2; % Number of all observation that are gained by group 1 of robots
            num_obs_by_group2 = num_rob_in_group2*N_L2*2; % Number of all observation that are gained by group 2 of robots
            
            R_std_team = [];
            for i = 1:num_rob_in_group1 % ie for the first half of robots
                eta = ObservationModel_class.eta; %#ok<PROP>
                sigma_b = ObservationModel_class.sigma_b;%#ok<PROP>
                x = x_team(3*i-2:3*i);
                d = L_group1-repmat(x(1:2),1,N_L1);
                ranges=sqrt(d(1,:).^2+d(2,:).^2)';
                R_std_single_robot(1:2:2*N_L1-1)=eta(1)*ranges+sigma_b(1);%#ok<PROP>
                R_std_single_robot(2:2:2*N_L1)=eta(2)*ranges+sigma_b(2);%#ok<PROP>
                R_std_team = [R_std_team , R_std_single_robot]; %#ok<AGROW>
            end
            for i = num_rob_in_group1+1:n % ie for the second half of robots
                eta = ObservationModel_class.eta; %#ok<PROP>
                sigma_b = ObservationModel_class.sigma_b;%#ok<PROP>
                x = x_team(3*i-2:3*i);
                d = L_group2-repmat(x(1:2),1,N_L2);
                ranges=sqrt(d(1,:).^2+d(2,:).^2)';
                R_std_single_robot(1:2:2*N_L2-1)=eta(1)*ranges+sigma_b(1);%#ok<PROP>
                R_std_single_robot(2:2:2*N_L2)=eta(2)*ranges+sigma_b(2);%#ok<PROP>
                R_std_team = [R_std_team , R_std_single_robot]; %#ok<AGROW>
            end
            R=diag(R_std_team.^2);
        end
        function innov = compute_innovation(Xprd,Zg)
            V = zeros(ObservationModel_class.obsNoiseDim,1);
            Zprd = ObservationModel_class.h_func(Xprd,V);
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
%             if ~isempty(user_data_class.par.figure_position)
%                 set(gcf,'Position',user_data_class.par.figure_position)
%             end
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