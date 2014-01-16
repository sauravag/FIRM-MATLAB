%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Steer heading
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function d_yaw_seq = steerHeading(psi_init,psi_final,n_steps,params)

% Heading Control sub-system (states: psi and r)

A = [0,1;0 0];
B = [0;1/(params.Iz)];

lower = [-pi/2;-pi;-1];
upper = [pi/2;pi;1];

x_init = [psi_init,0];
x_final = [psi_final,0];


[~,d_yaw_seq] = steerLinearSystembyLP(A,B,lower,upper,x_init,x_final,params.dt,n_steps);




end