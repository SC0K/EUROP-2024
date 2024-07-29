%% MPC Example with instial states and system as constraints
yalmip('clear')
clear all

% Model data
A = [2 -1;1 0.2];
B = [1;1];
nx = 2; % Number of states
nu = 1; % Number of inputs

% MPC data
Q = eye(2);
R = 2;
N = 7;

% Initial state
x0 = [0;0];
xt = [-5;1];

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));

constraints = [];
objective = 0;
% x = x0;
for k = 1:N
%  x = A*x + B*u{k};
constraints = [constraints, x{1}==x0];  % Initial states constraints
 constraints = [constraints, x{k+1} == A*x{k} + B* u{k}];
 objective = objective + norm(Q*(x{k} - xt),1) + norm(R*u{k},1);
 constraints = [constraints, -1 <= u{k}<= 1, -5*ones(nx,1)<=x{k}<=5*ones(nx,1)];
end

optimize(constraints,objective);
value(u{1})