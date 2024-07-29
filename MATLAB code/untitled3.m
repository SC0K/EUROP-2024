yalmip('clear')
clear all

h = 0.1;       % Simulation sample time (s)
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
T = 100; % Number of time steps

m = 1; % Number of scenarios
r1 = 0.25;     % Drone proximity limits
r2 = 0.5;
gamma = 0.2;
a_lim = 4;  % acceleration limit m/s^2

% 4 Drones
X = zeros(nx, nd, T+1);  % MegaState matrix (current states, 1 column, drone index, Time step)

for s = 1:m
    A{s} = A0;
    B{s} = B0;
end

% Initialize states
X(:,1,1) = [0;1;0;0];
X(:,2,1) = [0;-1;0;0];
X(:,3,1) = [1;0;0;0];
X(:,4,1) = [-1;0;0;0];

%% MPC data 
Q = 5*eye(nx);
R = 2*eye(nu);
for s = 1:m
    QN{s} = idare(A{s},B{s},Q,R,[],[]);  % Solution of Riccati equation
end
eta = 0.1;
N = 5;    % MPC Horizon

% Define state and input variables for MPC
for s = 1:m
    for d = 1:nd
        x_mega{s}{d} = sdpvar(nx, N);  % State variable for drone d
        u_mega{s}{d} = sdpvar(nu, N-1); % Input variable for drone d
    end
end

% Initialize constraints and objectives for each drone
constraints = cell(nd, 1);
objective = cell(nd, 1);
for d = 1:nd
    constraints{d} = [];
    objective{d} = 0;
end

for d = 1:nd
    for s = 1:m
        for k = 1:N-1
            % System dynamics constraints
            constraints{d} = [constraints{d}, x_mega{s}{d}(:,k+1) == A{s}*x_mega{s}{d}(:,k) + B{s}*u_mega{s}{d}(:,k)];
            constraints{d} = [constraints{d}, -a_lim*ones(nu,1) <= u_mega{s}{d}(:,k) <= a_lim*ones(nu,1)];
            
            % Objective function
            if mod(d, 2) == 0
                objective{d} = objective{d} + (x_mega{s}{d}(:,k)-X(:,d-1,1))'*Q*(x_mega{s}{d}(:,k)-X(:,d-1,1)) + u_mega{s}{d}(:,k)'*R*u_mega{s}{d}(:,k);
            else
                objective{d} = objective{d} + (x_mega{s}{d}(:,k)-X(:,d+1,1))'*Q*(x_mega{s}{d}(:,k)-X(:,d+1,1)) + u_mega{s}{d}(:,k)'*R*u_mega{s}{d}(:,k);
            end
        end
        % Terminal state cost
        if mod(d, 2) == 0
            objective{d} = objective{d} + eta*(x_mega{s}{d}(:,N)-X(:,d-1,1))'*QN{s}*(x_mega{s}{d}(:,N)-X(:,d-1,1));
        else
            objective{d} = objective{d} + eta*(x_mega{s}{d}(:,N)-X(:,d+1,1))'*QN{s}*(x_mega{s}{d}(:,N)-X(:,d+1,1));
        end
    end
end

%% CBF data
combinations = nchoosek(1:nd, 2);
[lx, ly] = size(combinations);
for c = 1:lx
    for s = 1:m
        for k = 1:N-1
            % CBF constraints
            hk = abs(x_mega{s}{combinations(c,1)}(1,k)-x_mega{s}{combinations(c,2)}(1,k))/r1 + abs(x_mega{s}{combinations(c,1)}(2,k)-x_mega{s}{combinations(c,2)}(2,k))/r2 - 1;
            hk1 = abs(x_mega{s}{combinations(c,1)}(1,k+1)-x_mega{s}{combinations(c,2)}(1,k+1))/r1 + abs(x_mega{s}{combinations(c,1)}(2,k+1)-x_mega{s}{combinations(c,2)}(2,k+1))/r2 - 1;
            constraints{combinations(c,1)} = [constraints{combinations(c,1)}, 0 <= hk1 - hk + gamma*hk];
            constraints{combinations(c,2)} = [constraints{combinations(c,2)}, 0 <= hk1 - hk + gamma*hk];
        end
        hk0 = abs(x_mega{s}{combinations(c,1)}(1,1)-x_mega{s}{combinations(c,2)}(1,1))/r1 + abs(x_mega{s}{combinations(c,1)}(2,1)-x_mega{s}{combinations(c,2)}(2,1))/r2 - 1;
        constraints{combinations(c,1)} = [constraints{combinations(c,1)}, 0 <= hk0];
        constraints{combinations(c,2)} = [constraints{combinations(c,2)}, 0 <= hk0];
    end
end

% Define the optimizer
for d = 1:nd
    U = [];
    for n = 1:N-1
        U = [U; u_mega{1}{d}(:,n)];
    end
    controller{d} = optimizer(constraints{d}, objective{d}, [], x_mega{1}{d}(:,1), U);
end

%% Simulation
for t = 1:T
    for d = 1:nd
        U = controller{d}(X(:,d,t));
        X(:,d,t+1) = A0*X(:,d,t) + B0*U(1:2);
    end
    t
end

% Create a new figure
figure;
hold on;

% Plot the trajectory of each drone with lines
for drone_idx = 1:nd
    % Extract the x and y positions of the drone over time
    x_positions = squeeze(X(1, drone_idx, :));  % x positions
    y_positions = squeeze(X(2, drone_idx, :));  % y positions
    
    % Plot the trajectory with lines
    plot(x_positions, y_positions, '-', 'DisplayName', ['Drone ' num2str(drone_idx)]);
end

% Plot initial and final positions with distinct markers
for drone_idx = 1:nd
    % Extract the initial and final positions
    initial_x = X(1, drone_idx, 1);
    initial_y = X(2, drone_idx, 1);
    final_x = X(1, drone_idx, T+1);
    final_y = X(2, drone_idx, T+1);
    
    % Plot the initial position
    plot(initial_x, initial_y, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', ['Start Drone ' num2str(drone_idx)]);
    
    % Plot the final position
    plot(final_x, final_y, 'x', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', ['End Drone ' num2str(drone_idx)]);
end

% Add labels and legend
xlabel('X Position');
ylabel('Y Position');
title('Trajectories of Drones');
legend('show');
grid on;
hold off;
