% Version 5 - adding CBF to the MPC - Optimizer version

yalmip('clear')
clear all

h = 0.3;       % Simulation sample time (s)
A0 = [1 0 h 0;
      0 1 0 h;
      0 0 1 0;
      0 0 0 1];
B0 = [h^2/2 0;
      0 h^2/2;
      h 0;
      0 h];
nx = 4; % Number of states
nu = 2; % Number of inputs
nd = 4; % Number of drones
T = 70; % Number of time steps

m = 1; % Number of scenarios
r1 = 0.25;     % Drone proximity limits
r2 = 0.5;
gamma = 0.2;
a_lim = 4;  % acceleration limit m/s^2

% 4 Drones
X = zeros(nx, nd, T+1);  % MegaState matrix (current states, drone index, Time step)

for s = 1:m
    A{s} = A0;
    B{s} = B0;
end

% Initialize states
X(:,1,1) = [0;1;0;0];
X(:,2,1) = [0;-1;0;0];
X(:,3,1) = [1;0;0;0];
X(:,4,1) = [-1;0;0;0];

% Target destinations
targets = [0 -1 0 0;
           0 1 0 0;
           -1 0 0 0;
           1 0 0 0];
targets = targets';

combinations = [2,3,4;
                1,3,4;
                1,2,4;
                1,2,3];
[lx,ly] = size(combinations);

%% MPC data 
Q = 5*eye(nx);
R = 2*eye(nu);
eta = 0.1;
N = 3;  % MPC Horizon



for d = 1:nd
    for s = 1:m
        QN = idare(A{s}, B{s}, Q, R, [], []);
        % Define the optimization variables and parameters
        x = sdpvar(nx,nd, N+1);     
        u = sdpvar(nu, N);      
        
        % Constraints and objective
        constraints = [x(:,1) == X(:,d,1)];
        objective = 0;
        for k = 1:N
            constraints = [constraints, x(:,k+1) == A{s}*x(:,k) + B{s}*u(:,k)];
            constraints = [constraints, -a_lim <= u(:,k) <= a_lim];
            objective = objective + (x(:,k)-targets(:,d))'*Q*(x(:,k)-targets(:,d)) + u(:,k)'*R*u(:,k);

%             for c = 1:ly
%                 hk = abs(x(1,k) - X(1,combinations(d,c),1))/r1 + abs(x(2,k) - X(2,combinations(d,c),1))/r2 - 1;
%                 hk1 = abs(x(1,k+1) - X(1,combinations(d,c),1))/r1 + abs(x(2,k+1) - X(2,combinations(d,c),1))/r2 - 1;
%                 constraints = [constraints, 0 <= hk1 - hk + gamma*hk];
%             end
        end

        objective = objective + eta*(x(:,N+1)-targets(:,d))'*QN*(x(:,N+1)-targets(:,d));
    end
end

% Create optimizer
parameters_in = {x(:,1)};
solutions_out = {u(:,1)};
controller = optimizer(constraints, objective, [], parameters_in, solutions_out);

%% Simulation
for t = 1:T
    for d = 1:nd
        % Compute the optimal control action
        U = controller{X(:,d,t)};
        % Apply the control action
        X(:,d,t+1) = A0*X(:,d,t) + B0*U;
    end
end

%% Plotting Trajectories
figure;
hold on;

% Plot the trajectory of each drone with lines
for drone_idx = 1:nd
    % Extract the x and y positions of the drone over time
    x_positions = squeeze(X(1, drone_idx, :));  % x positions
    y_positions = squeeze(X(2, drone_idx, :));  % y positions
    
    % Plot the trajectory with lines
    plot(x_positions, y_positions, '-', 'LineWidth', 3,'DisplayName', ['Drone ' num2str(drone_idx)]);
end

% Plot initial and final positions with distinct markers
for drone_idx = 1:nd
    % Extract the initial and final positions
    initial_x = X(1, drone_idx, 1);
    initial_y = X(2, drone_idx, 1);
    final_x = X(1, drone_idx, T+1);
    final_y = X(2, drone_idx, T+1);
    
    % Plot the initial position
    plot(initial_x, initial_y, 's', 'MarkerSize', 15, 'DisplayName', ['Initial Position Drone ' num2str(drone_idx)]);
    
    % Plot the final position
    plot(final_x, final_y, 'd', 'MarkerSize', 15, 'DisplayName', ['Final Position Drone ' num2str(drone_idx)]);
end

% Add labels and legend
xlabel('X Position');
ylabel('Y Position');
title('Trajectories of Drones');
legend('show');
grid on;
hold off;
