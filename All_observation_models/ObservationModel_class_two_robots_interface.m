classdef ObservationModel_class_two_robots_interface < ObservationModel_interface
    properties (Constant = true) % Note that you cannot change the order of the definition of properties in this class due to its ugly structure!! (due to dependency between properties.)
%         tmp_prop = ObservationModel_class.get_landmarks();  % This property is not needed!! I only added it because I could not find any other way to initialize the rest of constant properties. 
        Lx = 60;
        Ly = 60;
        tmp_prop = ObservationModel_class_two_robots_interface.draw();
        eta_x = 0.11*5;
        eta_y = 0.11*5;
        sigma_b_x = 0.2*2;
        sigma_b_y = 0.2*2;
        sigma_theta = 0;
        alpha_ratio = 10;
    end
    properties (Constant, Abstract)
        obsDim;
        obsNoiseDim; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
    end
    
    methods (Static)
        function handle_of_plot = draw()
            old_prop = ObservationModel_class_two_robots_interface.set_figure();
            a = axis;
            handle_of_plot = [];
            handle_of_plot = [handle_of_plot , plot([a(1),a(2)],[ObservationModel_class_two_robots_interface.Ly,ObservationModel_class_two_robots_interface.Ly],...
                'g--','linewidth',5) ];
            handle_of_plot = [handle_of_plot , plot([ObservationModel_class_two_robots_interface.Lx,ObservationModel_class_two_robots_interface.Lx],[a(3),a(4)],...
                'g--','linewidth',5) ];
           ObservationModel_class_two_robots_interface.reset_figure(old_prop);
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
    methods (Static, Abstract)
        z_team = h_func(x_team,v_team)
        H_team = dh_dx_func(x_team,v_team) 
        M = dh_dv_func(x,v) 
        V = generate_observation_noise(x)
        R = noise_covariance(x_team)
    end
end