%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continous quadrotor dynamics,
% gives xdot for a given x and u
% 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xdot = quadDyn(x,u,params)

% The state vector alphabet

px = x(1);
py = x(2);
pz = x(3);
pxdot = x(4);
pydot = x(5);
pzdot = x(6);

phi = x(7);
theta = x(8);
psi = x(9);
p = x(10);
q = x(11);
r = x(12);

% The input vector alphabet

d_collective = u(1);
d_roll = u(2);
d_pitch = u(3);
d_yaw = u(4);

% parameters

g = params.g;
Ix = params.Ix;
Iy = params.Iy;
Iz = params.Iz;
L = params.L;
m = params.m;

% Dynamics

xdot = zeros(12,1);


xdot(1) = pxdot;
xdot(2) = pydot;
xdot(3) = pzdot;

xdot(4) = g*phi;
xdot(5) = -g*theta;
xdot(6) = d_collective/m;

xdot(7) = p;
xdot(8) = q;
xdot(9) = r;

xdot(10) = L*d_roll/Ix;
xdot(11) = L*d_pitch/Iy;
xdot(12) = 1*d_yaw/Iz;

end