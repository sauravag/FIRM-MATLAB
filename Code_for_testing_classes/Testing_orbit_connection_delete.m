clc;clear variables;close all

addpath('All_motion_models\Unicycle_robot\')

T = 20;
th_steps = 2*pi/T;

radius = 10; % radius of the circular orbits
omega_p = th_steps/MotionModel_class.dt * ones(1,T);
V_p = omega_p * radius;

u_p = [V_p;-omega_p];
w = zeros(MotionModel_class.wDim,T); % no noise
v = zeros(ObservationModel_class.obsNoiseDim,T); % no noise

center(:,1) = [85 ; 15];
center(:,2) = [27 ; 7];
center(:,3) = [20 ; 62];
center(:,4) = [70 ; 67];
center(:,5) = [52 ; 36];

num_orbits = size(center,2);

figure(gcf); hold on; grid on; set(gca, 'DrawMode', 'fast'); set(gca,'DataAspectRatio',[1 1 1])
for j = 1 : num_orbits
    clear x_p
    x_p(:,1) = [center(:,j) - [0;radius] ; 0*pi/180]; % initial x
    orbit_center = center(:,j); % This is only true when teh initial angle is zero! which is the case here.
    % Drawing orbit
    th_orbit_draw = 0:0.1:2*pi;
    x_orbit_draw = orbit_center(1) + radius*cos(th_orbit_draw);
    y_orbit_draw = orbit_center(2) + radius*sin(th_orbit_draw);
    plot(x_orbit_draw,y_orbit_draw)
    
    for k=1:T
        Xstate = state(x_p(:,k));
        Xstate.draw('RobotShape','triangle','robotsize',1);%,'TriaColor',color(cycles));
        linear_pts(k).x = x_p(:,k); linear_pts(k).u = u_p(:,k); linear_pts(k).w = w(:,k); linear_pts(k).v = v(:,k);
        x_p(:,k+1) = MotionModel_class.f_discrete(x_p(:,k),u_p(:,k),w(:,k));
    end
    plot(x_p(1,:),x_p(2,:),'k')
    
end

for j = 1 : num_orbits

    %=============================== System Linearization
    for k=1:T
        LSS(k) = Linear_system_class(linear_pts(k));
    end
    %=============================== Periodic Kalman Filter
    [KG,Pprd,Pest ] = Kalman_filter.periodic_gain_and_covariances(LSS);
    
    for k = 1:T
%         plotUncertainEllip2D(Pprd(1:2,1:2,k),x_p(1:2,k),'b', 2);
        plotUncertainEllip2D(Pest(1:2,1:2,k),x_p(1:2,k),'r', 2);
        drawnow
    end
    %=============================== Periodic LQR
    
    FG = LQR_class.periodic_gains(LSS);
    
    %=============================== Periodic LQG
    orbit(j).x = x_p(:,1:T);  % "x_p" is of length T+1, but "x_p(:,T+1)" is equal to "x_p(:,1)"
    orbit(j).u = u_p;  % "u_p" is of length T.
    orbit(j).center = center(:,j);
    orbit.radius = radius;
    
    node_controller = LQG_Periodic_class(orbit(j));
    
%     Xg = state(x_p(:,1) + 0 * randn);
%     est_mean =Xg;
%     est_cov = 1 * Pest(:,:,1);
%     b = belief(est_mean, est_cov);
%     Hstate_periodic(1) = Hstate(Xg,b);
%     
%     
%     NP = 5;
%     for ppp = 1:NP
%         for k = 1:T
%             Hstate_periodic(k) = Hstate_periodic(k).draw('EllipseSpec','g','EllipseWidth',2);
% %             if ppp>1, Hstate_periodic(k+1) = Hstate_periodic(k+1).delete_plot('EllipseSpec','g','EllipseWidth',2); end
%             [Hstate_periodic(k+1), reliable] = node_controller.propagate_Hstate(Hstate_periodic(k),k,'No-noise');
%             drawnow
%         end
%         Hstate_periodic(1) = Hstate_periodic(1).delete_plot('EllipseSpec','g','EllipseWidth',2);
%         Hstate_periodic(1) =  Hstate_periodic(T+1);
%     end
    
%=============================== FIRM Node Test
FN = FIRM_node_class(orbit (j), 1, ceil(T/2), 1);
FN = FN.construct_node();
FN = FN.draw('EllipseSpec','k','EllipseWidth',3);

for p_n = 1 : user_data_class.par.par_n
    Xg = state(x_p(:,1) + (p_n - 1)*randn);
    est_mean =Xg;
    est_cov = (p_n) * Pest(:,:,1);
    b = belief(est_mean, est_cov);
    Hstate_particles(p_n) = Hstate(Xg,b);
end
PHb(1) = Hbelief_p(Hstate_particles);

FN.construct_seq_of_Hb(PHb);

end
% xlim([0 100])
% ylim([-10 82])
set(gca,'DataAspectRatio',[1 1 1])
