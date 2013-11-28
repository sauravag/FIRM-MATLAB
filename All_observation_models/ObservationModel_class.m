% Observation Model for 6 Dof Flying Robot with radio beacon based observations 
% The beacons can be anywhere in 3D space.
% Quaternions are used to represent the orientation of the robot

classdef ObservationModel_class < ObservationModel_interface
    % Note that because the class is defined as a handle class, the
    % properties must be defined such that they are do not change from an
    % object to another one.
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its structure (due to dependency between properties.)

        tmp_prop = ObservationModel_class.costant_property_constructor();  % I use this technique to initialize the costant properties in run-time. If I can find a better way to do it, I will update it, as it seems a little bit strange.
        landmarks = ObservationModel_class.tmp_prop.landmarks;
        obsDim = ObservationModel_class.tmp_prop.obsDim;
        obsNoiseDim = ObservationModel_class.obsDim; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
        zeroNoise = zeros(ObservationModel_class.obsNoiseDim,1); % zero observation noise
        eta = [0.0;0.0;0.0];%user_data_class.par.observation_model_parameters.eta; 
        sigma_b = [0.01;deg2rad(0.1);deg2rad(0.1)];%user_data_class.par.observation_model_parameters.sigma_b;
    end
    properties
       plot_handle;
    end
    
    methods (Static = true)
        function temporary_props = costant_property_constructor()
            LoadFileName = user_data_class.par.LoadFileName;
            SaveFileName = user_data_class.par.SaveFileName;
            Man_L = user_data_class.par.observation_model_parameters.interactive_OM;
            if Man_L == 0
                load(LoadFileName,'Landmarks')
                temporary_props.landmarks = Landmarks; %#ok<NODEF>
                temporary_props.landmarks(3,:) = 0; % Z coordinate is 0
                temporary_props.obsDim =  3*size(Landmarks,2);
            else
                temporary_props = ObservationModel_class.request_landmarks();
            end
            Landmarks = temporary_props.landmarks; %#ok<NASGU>
            save(SaveFileName,'Landmarks','-append') % here, we save the landmarks for the future runs.
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
            Landmarks=[Lx;Ly;zeros(1,size(Lx,2))];
            temporary_props.landmarks = Landmarks;
            temporary_props.obsDim = 3*size(Landmarks,2);
            ObservationModel_class.reset_figure(old_prop);
            title([])
        end
        
        function z = h_func(x,v)
            L=ObservationModel_class.landmarks;
            od = ObservationModel_class.obsDim;
            N_L=size(L,2);
                        
            d=L-repmat(x(1:3),1,N_L);
            
            z(1:3:od-2,1) = sqrt(d(1,:).^2+d(2,:).^2+d(3,:).^2)'+v(1:3:od-2,1);
            
            q = [x(4) x(5) x(6) x(7)]; % compose the quaternion
            
            R = quat2dcm(q); % get the DCM from the quaternion
            
            d_body = R*d; % rotate the relative vectors to body frame;
            
            z(2:3:od-1,1) = atan2(d_body(2,:),d_body(1,:))' + v(2:3:od-1,1); % Azimuth
             
            z(3:3:od,1) = atan2(d_body(3,:),d_body(1,:))' + v(3:3:od,1); % Elevation
            
        end
        
        function H = dh_dx_func(x,v) %#ok<INUSD>
            L = ObservationModel_class.landmarks;
            od = ObservationModel_class.obsDim;
            H = nan(od,state.dim); % memory preallocation
            
            q = [x(4) x(5) x(6) x(7)];
            q0 = q(1);
            q1 = q(2);
            q2 = q(3);
            q3 = q(4);
            
            R = quat2dcm(q) ; % R rotates from inertial to body frame
            
            dR_by_dq0 = 2*[q0,q3,-q2;-q3,q0,q1;q2,-q1,q0];
            
            dR_by_dq1 = 2*[q1,q2,q3;q2,-q1,q0;q3,-q0,-q1];
            
            dR_by_dq2 = 2*[-q2,q1,-q0;q1,q2,q3;q0,q3,-q2];
          
            dR_by_dq3 = 2*[-q3,q0,q1;-q0,-q3,q2;q1,q2,q3];
            
            for i=1:size(L,2)
                d_ig = L(:,i)- x(1:3); % displacement between robot and landmark in ground frame
                d_ib = R*d_ig ; % displacement in body frame
                r_i = norm(d_ig); % scalar distance between landmark 'i' and robot
                
                Hi_11 = -d_ig(1) / r_i;
                Hi_12 = -d_ig(2) / r_i;
                Hi_13 = -d_ig(3) / r_i;
                Hi_14 = 0;
                Hi_15 = 0;
                Hi_16 = 0;
                Hi_17 = 0;
                
                temp1 = 1 / ( (d_ib(1))^2 + (d_ib(2))^2 );
                
                dx_ib_by_dq0 = dR_by_dq0(1,1)*d_ig(1) + dR_by_dq0(1,2)*d_ig(2) + dR_by_dq0(1,3)*d_ig(3);
                dx_ib_by_dq1 = dR_by_dq1(1,1)*d_ig(1) + dR_by_dq1(1,2)*d_ig(2) + dR_by_dq1(1,3)*d_ig(3);
                dx_ib_by_dq2 = dR_by_dq2(1,1)*d_ig(1) + dR_by_dq2(1,2)*d_ig(2) + dR_by_dq2(1,3)*d_ig(3);
                dx_ib_by_dq3 = dR_by_dq3(1,1)*d_ig(1) + dR_by_dq3(1,2)*d_ig(2) + dR_by_dq3(1,3)*d_ig(3);
                
                dy_ib_by_dq0 = dR_by_dq0(2,1)*d_ig(1) + dR_by_dq0(2,2)*d_ig(2) + dR_by_dq0(2,3)*d_ig(3);
                dy_ib_by_dq1 = dR_by_dq1(2,1)*d_ig(1) + dR_by_dq1(2,2)*d_ig(2) + dR_by_dq1(2,3)*d_ig(3);
                dy_ib_by_dq2 = dR_by_dq2(2,1)*d_ig(1) + dR_by_dq2(2,2)*d_ig(2) + dR_by_dq2(2,3)*d_ig(3);
                dy_ib_by_dq3 = dR_by_dq3(2,1)*d_ig(1) + dR_by_dq3(2,2)*d_ig(2) + dR_by_dq3(2,3)*d_ig(3);
                
                dz_ib_by_dq0 = dR_by_dq0(3,1)*d_ig(1) + dR_by_dq0(3,2)*d_ig(2) + dR_by_dq0(3,3)*d_ig(3);
                dz_ib_by_dq1 = dR_by_dq1(3,1)*d_ig(1) + dR_by_dq1(3,2)*d_ig(2) + dR_by_dq1(3,3)*d_ig(3);
                dz_ib_by_dq2 = dR_by_dq2(3,1)*d_ig(1) + dR_by_dq2(3,2)*d_ig(2) + dR_by_dq2(3,3)*d_ig(3);
                dz_ib_by_dq3 = dR_by_dq3(3,1)*d_ig(1) + dR_by_dq3(3,2)*d_ig(2) + dR_by_dq3(3,3)*d_ig(3);
                
                Hi_21 = temp1*(-R(2,1)*d_ib(1) + R(1,1)*d_ib(2)) ;
                Hi_22 = temp1*(-R(2,2)*d_ib(1) + R(1,2)*d_ib(2)) ;
                Hi_23 = temp1*(-R(2,3)*d_ib(1) + R(1,3)*d_ib(2)) ;
                Hi_24 = temp1*(d_ib(1)*dy_ib_by_dq0 -d_ib(2)*dx_ib_by_dq0);
                Hi_25 = temp1*(d_ib(1)*dy_ib_by_dq1 -d_ib(2)*dx_ib_by_dq1);
                Hi_26 = temp1*(d_ib(1)*dy_ib_by_dq2 -d_ib(2)*dx_ib_by_dq2);
                Hi_27 = temp1*(d_ib(1)*dy_ib_by_dq3 -d_ib(2)*dx_ib_by_dq3);
                
                temp2 =  1 / ( (d_ib(1))^2 + (d_ib(3))^2 );
                
                Hi_31 = temp2 *(-R(3,1)*d_ib(1) + R(1,1)*d_ib(3));
                Hi_32 = temp2 *(-R(3,2)*d_ib(1) + R(1,2)*d_ib(3));
                Hi_33 = temp2 *(-R(3,3)*d_ib(1) + R(1,3)*d_ib(3));
                Hi_34 = temp2*(d_ib(1)*dz_ib_by_dq0 -d_ib(3)*dx_ib_by_dq0);
                Hi_35 = temp2*(d_ib(1)*dz_ib_by_dq1 -d_ib(3)*dx_ib_by_dq1);
                Hi_36 = temp2*(d_ib(1)*dz_ib_by_dq2 -d_ib(3)*dx_ib_by_dq2);
                Hi_37 = temp2*(d_ib(1)*dz_ib_by_dq3 -d_ib(3)*dx_ib_by_dq3);
                
                 
                H(3*i-2:3*i,:) = [ Hi_11,Hi_12,Hi_13,Hi_14,Hi_15,Hi_16,Hi_17;
                                   Hi_21,Hi_22,Hi_23,Hi_24,Hi_25,Hi_26,Hi_27;
                                   Hi_31,Hi_32,Hi_33,Hi_34,Hi_35,Hi_36,Hi_37];
                                   
            end
        end
        function M = dh_dv_func(x,v) %#ok<INUSD>
            % Jacobian of observation wrt observation noise.
            M = eye(ObservationModel_class.obsDim);
        end
        function V = generate_observation_noise(x)
            L = ObservationModel_class.landmarks;
            eta = ObservationModel_class.eta; %#ok<PROP>
            sigma_b = ObservationModel_class.sigma_b;%#ok<PROP>
            obsDim = ObservationModel_class.obsDim;%#ok<PROP>
            N_L = size(L,2);
            
            d = L - repmat(x(1:3),1,N_L);
            ranges = sqrt(d(1,:).^2+d(2,:).^2+d(3,:).^2)';
            
            R_std(1:3:obsDim-2) = eta(1)*ranges+sigma_b(1);%#ok<PROP>
            R_std(2:3:obsDim-1) = eta(2)*ranges+sigma_b(2);%#ok<PROP>
            R_std(3:3:obsDim)   = eta(3)*ranges+sigma_b(3);%#ok<PROP>
            R = diag(R_std.^2);
            
            indep_part_of_obs_noise=randn(obsDim,1); %#ok<PROP>
            V = indep_part_of_obs_noise.*diag(R.^(1/2));
        end
        function R = noise_covariance(x)
            od = ObservationModel_class.obsDim;
            L = ObservationModel_class.landmarks;
            eta = ObservationModel_class.eta; %#ok<PROP>
            sigma_b = ObservationModel_class.sigma_b;%#ok<PROP>
            N_L = size(L,2);
            d = L - repmat(x(1:3),1,N_L);
            ranges = sqrt(d(1,:).^2+d(2,:).^2+d(3,:).^2)';
            R_std(1:3:od-2) = eta(1)*ranges+sigma_b(1);%#ok<PROP>
            R_std(2:3:od-1) = eta(2)*ranges+sigma_b(2);%#ok<PROP>
            R_std(3:3:od)   = eta(3)*ranges+sigma_b(3);%#ok<PROP>
            R=diag(R_std.^2);
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
                if mod(i,3)==0 && innov(i)>pi
                    innov(i)=innov(i)-2*pi;
                elseif mod(i,3)==0 && innov(i)<-pi
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
            if ~isempty(user_data_class.par.sim.figure_position)
                set(gcf,'Position',user_data_class.par.sim.figure_position)
            end
            axis(user_data_class.par.sim.env_limits);
            set(gca,'DataAspectRatio',[1 1 1]); % makes the scaling of different axes the same. So, a circle is shown as a circle not ellipse.
        end
        function reset_figure(old_prop) % This function resets the figure properties (size and other properties), to what they were before setting them in this class.
            set(gca,'NextPlot',old_prop{1}); % reset the "NextPlot" property to what it was.
            set(gca,'XGrid',old_prop{2}); % reset  the "XGrid" property to what it was.
            set(gca,'YGrid',old_prop{3}); % reset  the "YGrid" property to what it was.
        end
    end
    
    methods
        function obj = draw(obj) % note that the "draw" function in this class is "static". Thus, if you call it, you have to assign its output to the "plot_handle" by yourself.
            old_prop = ObservationModel_class.set_figure();
            obj.plot_handle = plot(obj.landmarks(1,:),obj.landmarks(2,:),'kp','markerfacecolor','k','markersize',12);
            ObservationModel_class.reset_figure(old_prop);
        end
        function obj = delete_plot(obj)
            delete(obj.plot_handle)
            obj.plot_handle = [];
        end
    end
end