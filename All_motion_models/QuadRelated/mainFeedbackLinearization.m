%% Referesh 

clear all;
close all;
clc;



%% Enter Parameters

params.g = 9.81; % Mass kg

params.m = 0.650; % quadrotor mass kg
params.Ix = 7.5e-3; % Moment of Inertia kg*m2
params.Iy = 7.5e-3; % Moment of Inertia kg*m2
params.Iz = 1.3e-2; % Moment of Inertia kg*m2
params.L = 0.23; % Arm Length m

dt = 0.1; % timestep (seconds)

params.dt = dt;

%% Initial and Final States


% Initial State

p_init = rand(3,1)*100;
pdot_init = [0;0;0];

att_init = [0;0;0];
attdot_init = [0;0;0];

x_init = [p_init;pdot_init;att_init;attdot_init];


% Final State

p_final = (rand(3,1)-0.5)*env_size;
pdot_final = [0;0;0];

% att_final = [0;0;pi/3];
att_final = [0;0;0];
attdot_final = [0;0;0];

x_final = [p_final;pdot_final;att_final;attdot_final];

n_steps = 30;

% Reference States (X,Xdot and Y,Ydot)

% Fit a second order polynomial path

delta_x = p_final(1) -p_init(1);

time = 0:dt:(n_steps-1)*dt;

refs = zeros(8,n_steps);

refs(1,:) = p_final(1); % px command
refs(5,:) = p_final(2); % py command


% for i=1:floor(n_steps/2)
%    
%     refs(1,i) = p_init(1) + 2*delta_x/(n_steps*dt)*time(i)^2/2;
%     refs(2,i) = 2*delta_x/(n_steps*dt)*time(i);
%     refs(3,i) = 2*delta_x/(n_steps*dt);
%     refs(4,i) = 0;
%     
%     
% end
% 
% 
% for i=floor(n_steps/2)+1:n_steps
%    
%     refs(1,i) = p_init(1) + 2*delta_x/(n_steps*dt)*time(i)^2/2;
%     refs(2,i) = 2*delta_x/(n_steps*dt)-2*delta_x/(n_steps*dt)*time(i);
%     refs(3,i) = -2*delta_x/(n_steps*dt);
%     refs(4,i) = 0;
%     
%     
% end


%% Propogate with FL Control laws

states = zeros(12,n_steps);

states(:,1) = x_init;

u = zeros(4,n_steps-1);

for time_index=2:n_steps
    
    % Control
    
    u(:,time_index-1) =  QuadFeedbackLinearization(states(:,time_index-1),...
        refs(:,time_index-1),params);
    
    
    % Integration
    states(:,time_index) =... 
    quadDyn(states(:,time_index-1),u(:,time_index-1),params)*params.dt +...
    states(:,time_index-1);
    
end


%% Plot X and Y States

%time = (1:n_steps)*params.dt;

figure(1)
subplot(3,2,1)
plot(time,states(1,:));
subplot(3,2,2)
plot(time,states(2,:));
subplot(3,2,3)
plot(time,states(7,:));
subplot(3,2,4)
plot(time,states(8,:));
subplot(3,2,5)
plot(time(1:end-1),u(2,:));
subplot(3,2,6)
plot(time(1:end-1),u(3,:));








