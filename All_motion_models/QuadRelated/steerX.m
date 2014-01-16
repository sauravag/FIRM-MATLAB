%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Steer heading
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function  d_roll_seq = steerX(px_init,px_final,n_steps,params)

% X Control sub-system(states x xdot phi p)

A = zeros(4,4);
A(1,2) = 1;
A(2,3) = params.g;
A(3,4) = 1;

B = zeros(4,1);
B(4,1) = params.L/params.Ix;

lower = [-2;-2;-pi/2;-pi;-1];
upper = [2;2;pi/2;pi;1];

x_init = [px_init,0,0,0];
x_final = [px_final,0,0,0];

[~,d_roll_seq] = steerLinearSystembyLP(A,B,lower,upper,x_init,x_final,params.dt,n_steps);


end