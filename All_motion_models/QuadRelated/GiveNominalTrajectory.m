function traj = GiveNominalTrajectory(x_init,x_final)

%% Enter Parameters

params.g = 9.81; % Mass kg

params.m = 0.650; % quadrotor mass kg
params.Ix = 7.5e-3; % Moment of Inertia kg*m2
params.Iy = 7.5e-3; % Moment of Inertia kg*m2
params.Iz = 1.3e-2; % Moment of Inertia kg*m2
params.L = 0.23; % Arm Length m

dt = 0.1; % timestep (seconds)

params.dt = dt;

n_steps = 30;

time = 0:dt:(n_steps-1)*dt;

refs = zeros(10,n_steps);

refs(1,:) = x_final(1); % px command
refs(5,:) = x_final(2); % py command
refs(9,:) = x_final(3); % px command


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

%% Trajectory

traj.states = states;
traj.input = u;


%% Plot X and Y States

%time = (1:n_steps)*params.dt;

figure(1)
subplot(3,3,1)
plot(time,states(1,:));
ylabel('X Position');
subplot(3,3,2)
plot(time,states(2,:));
ylabel('Y Position');
subplot(3,3,3)
plot(time,states(3,:));
ylabel('Z Position');
subplot(3,3,4)
plot(time,states(7,:));
ylabel('Roll');
subplot(3,3,5)
plot(time,states(8,:));
ylabel('Pitch');
subplot(3,3,6)
plot(time,states(9,:));
ylabel('Yaw');
subplot(3,3,7)
plot(time(1:end-1),u(2,:));
ylabel('Roll Input');
subplot(3,3,8)
plot(time(1:end-1),u(3,:));
ylabel('Pitch Input');
subplot(3,3,9)
plot(time(1:end-1),u(1,:));
ylabel('Collective Input');










