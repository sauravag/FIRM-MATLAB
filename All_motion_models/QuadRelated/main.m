%% Referesh 

clear all;
close all;
clc;

%% Enter Parameters
% rand('state',20)
% randn('state',20)

params.g = 9.81; % Mass kg

params.m = 0.650; % quadrotor mass kg
params.Ix = 7.5e-3; % Moment of Inertia kg*m2
params.Iy = 7.5e-3; % Moment of Inertia kg*m2
params.Iz = 1.3e-2; % Moment of Inertia kg*m2
params.L = 0.23; % Arm Length m

params.dt = 0.1; % timestep (seconds)

%% Initial and Final States

% Initial State

env_size = 100;
% p_init = [0;0;1];
p_init = (rand(3,1)-0.5)*env_size;
pdot_init = [0;0;0];

att_init = [0;0;0];
attdot_init = [0;0;0];

x_init = [p_init;pdot_init;att_init;attdot_init];

% Final State

% p_final = [2;2;1];
 p_final = (rand(3,1)-0.5)*env_size;
pdot_final = [0;0;0];

% att_final = [0;0;pi/3];
att_final = [0;0;0];
attdot_final = [0;0;0];

x_final = [p_final;pdot_final;att_final;attdot_final];

%% Evaluate Steering function

%n_steps_heading= 10;
%n_steps_dir= 50;
%n_steps_total = n_steps_heading + n_steps_dir;
n_steps = 500;
% Steer heading

d_yaw_seq = steerHeading(att_init(3),att_final(3),n_steps,params);
d_roll_seq = steerX(p_init(1),p_final(1),n_steps,params);
d_pitch_seq= steerY(p_init(2),p_final(2),n_steps,params);

u = zeros(4,n_steps-1);
u(4,1:n_steps-1) = d_yaw_seq;
u(2,1:n_steps-1) = d_roll_seq;
u(3,1:n_steps-1) = d_pitch_seq;



%% Apply inputs
states = zeros(12,n_steps);

states(:,1) = x_init;

for time_index=2:n_steps
    
    % Integration
    states(:,time_index) =... 
    quadDyn(states(:,time_index-1),u(:,time_index-1),params)*params.dt +...
    states(:,time_index-1);
    
end

%% Plot kinematics

% External function trajectory2.m (taken from MATLAB File Exchange)

x = states(1,:);
y = states(2,:);
z= states(3,:);

% Ofset (optional)

%x(end) = 10;

pitch = states(7,:);
roll =  states(8,:);
yaw = states(9,:);

scale_factor = 1;
step = 2;
selector = 'helicopter';

trajectory2(x,y,z,pitch,roll,yaw,scale_factor,step,selector);
axis(env_size*[-.5 .5 -.5 .5 -.5 .5]);
set(gca,'DataAspectRatio',[1 1 1]); % makes the scaling of different axes the same. So, a circle is shown as a circle not ellipse.
view([30,40])