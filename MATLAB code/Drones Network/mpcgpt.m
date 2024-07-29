% Load YALMIP and Solver
yalmip('clear');

% Simulation parameters
h = 0.01; % Sample time
T = 100; % Number of time steps

% System matrices
A0 = [1 0 h 0; 0 1 0 h; 0 0 1 0; 0 0 0 1];
B0 = [h^2/2 0; 0 h^2/2; h 0; 0 h];

nx = 4; % Number of states
nu = 2; % Number of inputs
nd = 4; % Number of drones

% Initial and target positions
p0 = [0 0 1 -1; 1 -1 0 0];
pd = [0 0 1 -1; -1 1 0 0];

% Define prediction horizon and scenarios
N = 10; % Prediction horizon
m = nd; % Number of scenarios (drones)

% Define the weighting matrices
Q = eye(nx);
R = eye(nu);
Q_N = eye(nx);
eta = 1;

% Initialize YALMIP variables
x = sdpvar(nx, N+1, m); % State variable for m scenarios
u = sdpvar(nu, N, m);   % Control input for m scenarios

% Define disturbance matrix as zeros for this setup
d = zeros(nx, N, m);

% Initialize objective and constraints
objective = 0;
constraints = [];

for i = 1:m
    for k = 1:N
        % Define the cost function
        objective = objective + (x(:, k, i)' * Q * x(:, k, i) + u(:, k, i)' * R * u(:, k, i));
        
        % Define the system dynamics constraints
        constraints = [constraints, x(:, k+1, i) == A0 * x(:, k, i) + B0 * u(:, k, i)];
        
        % State and input constraints
        % Adjust according to your specific constraints, e.g.,
        % constraints = [constraints, x_min <= x(:, k, i) <= x_max];
        % constraints = [constraints, u_min <= u(:, k, i) <= u_max];
    end
    
    % Terminal cost
    objective = objective + eta * (x(:, N+1, i)' * Q_N * x(:, N+1, i));
    
    % Initial state constraint
    constraints = [constraints, x(:, 1, i) == [p0(:, i); zeros(nx-2, 1)]];
end

% Options for solver
options = sdpsettings('solver', 'quadprog', 'verbose', 0);

% Solve the optimization problem
controller = optimizer(constraints, objective, options, x(:, 1, :), u);

% Simulation parameters
x_sim = [p0; zeros(nx-2, nd)];
u_sim = zeros(nu, T, m);

for t = 1:T
    % Compute the optimal control action
    [u_opt, diagnostics] = controller(x_sim(:, :, end));
    
    if diagnostics == 1
        error('The problem is infeasible');
    end
    
    % Apply the control action
    for i = 1:m
        x_next(:, i) = A0 * x_sim(:, i, end) + B0 * u_opt(:, i);
    end
    x_sim = cat(3, x_sim, x_next);
    u_sim(:, t, :) = u_opt;
end

% Plot results
figure;
for i = 1:m
    subplot(2, 1, 1);
    plot(0:T, squeeze(x_sim(1, :, i)));
    hold on;
    title('State Trajectories');
    xlabel('Time Step');
    ylabel('States');
    legend('x_1', 'x_2');
    
    subplot(2, 1, 2);
    stairs(0:T-1, squeeze(u_sim(:, :, i)));
    hold on;
    title('Control Input');
    xlabel('Time Step');
    ylabel('u');
end
