function u = QuadFeedbackLinearization(state,ref,params)

u = zeros(4,1);

% Ref is px_ref,px_refd,px_refdd,px_refddd

% px_ref = ref(1);
% px_refdot = ref(2);
% px_refdd = ref(3);
% px_refddd = ref(4);

% State is 6DOF state vector

% Both systems have feedback linearization degree 4

% Roots of the linear systems (4th order)

roots = [-5+2i,-5-2i,-2,-4];

coeffs = poly(roots);

% Roots of the 2nd order system

roots2 = [-2+1i,-2-1i];
coeffs2 = poly(roots2);

% 
% px_ref_dots = [px_ref,px_refdot,0,0,0];
% py_ref_dots = [py_ref,py_refdot,0,0,0];

% States
px = state(1);
py = state(2);
pz = state(3);
pxdot = state(4);
pydot = state(5);
pzdot = state(6);

phi = state(7);
theta = state(8);
psi = state(9);
p = state(10);
q = state(11);
r = state(12);

% params
L = params.L;
Ix = params.Ix;
Iy = params.Iy;
g = params.g;
m = params.m;


% X Inversion

x_input_ps = 0 - coeffs(2)*(g*p - ref(4)) -...
     coeffs(3)*(g*phi - ref(3)) - coeffs(4)*(pxdot-ref(2))...
     - coeffs(5)*(px-ref(1));
u(2) = (Ix/(L*g))*(x_input_ps);

% Y Inversion

y_input_ps = 0 - coeffs(2)*(-g*q - ref(8)) -...
     coeffs(3)*(-g*theta - ref(7)) - coeffs(4)*(pydot-ref(6))...
     - coeffs(5)*(py-ref(5));
u(3) = -(Iy/(L*g))*(y_input_ps);

% Z Inversion

z_input_ps = 0 - coeffs2(2)*(pzdot-ref(10)) - coeffs2(3)*(pz-ref(9));
u(1) = 1/m*z_input_ps;

end
