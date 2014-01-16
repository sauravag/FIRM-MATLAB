%% Refresh
clc;
clear all;
close all;

%% Simple Linear System

% A = 1;
% B = 1;
% x_init = 0;
% x_final = 5;
% lower = [0;-1];
% upper = [6;1];
% dt = 0.1;
% numberOfSteps = 20;
% 
% [x_seq,u_seq] = steerLinearSystembyLP(A,B,lower,upper,x_init,x_final,dt,numberOfSteps);
% 
% [n,m] = size(B);
% X = zeros(n,numberOfSteps);
% X(:,1) = x_init;
% 
% for i=2:numberOfSteps
%    
%     X(:,i) = (A*X(:,i-1) +B*u_seq(:,i-1))*dt + X(:,i-1);
%     
% end
% 
% plot(0:dt:dt*(numberOfSteps-1),X);
% 
% %% Simple Double Integrator
% 
% A = [0,1;0 0];
% B = [0;1];
% 
% x_init = [0;0];
% x_final = [1;0];
% dt = 0.1;
% numberOfSteps = 10;
% 
% [x_seq,u_seq] = steerLinearSystembyLP(A,B,x_init,x_final,dt,numberOfSteps);
% 
% [n,m] = size(B);
% X = zeros(n,numberOfSteps);
% X(:,1) = x_init;
% 
% for i=2:numberOfSteps
%    
%     X(:,i) = (A*X(:,i-1) +B*u_seq(:,i-1))*dt + X(:,i-1);
%     
% end
% 
% plot(0:dt:dt*(numberOfSteps-1),X(1,:));


%% Steer Quadrotor

g = 9.81; % Mass kg

m = 0.650; % quadrotor mass kg
Ix = 7.5e-3; % Moment of Inertia kg*m2
Iy = 7.5e-3; % Moment of Inertia kg*m2
Iz = 1.3e-2; % Moment of Inertia kg*m2
L = 0.23; % Arm Length m

dt = 0.1; % timestep (seconds)

A = zeros(12,12);
B = zeros(12,4);
B = zeros(12,1);

A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

A(4,4) = g;
A(5,5) = -g;

A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

B(6,1) = 1/m;
B(10,2) = L/Ix;
B(11,3) = L/Iy;
B(12,4) = L/Iz;


% Initial State

p_init = [0;0;10];
pdot_init = [0;0;0];

att_init = [0;0;0];
attdot_init = [0;0;0];

x_init = [p_init;pdot_init;att_init;attdot_init];


% Final State

p_final = [1;0;10];
pdot_final = [0;0;0];

att_final = [0;0;0];
attdot_final = [0;0;0];

x_final = [p_final;pdot_final;att_final;attdot_final];

numberOfSteps = 100;

[x_seq,u_seq] = steerLinearSystembyLP(A,B,x_init,x_final,dt,numberOfSteps);

[n,m] = size(B);
X = zeros(n,numberOfSteps);
X(:,1) = x_init;

for i=2:numberOfSteps
   
    X(:,i) = (A*X(:,i-1) +B*u_seq(:,i-1))*dt + X(:,i-1);
    
end

plot(0:dt:dt*(numberOfSteps-1),X(1,:));

