yalmip('clear')
clear all

h = 0.05;       % Simulation sample time (s)
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
T = 300; % Number of time steps

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

%% MPC data 
Q = 5*eye(nx);
R = 2*eye(nu);
for s = 1:m
    QN{s} = idare(A{s}, B{s}, Q, R, [], []);  % Solution of Riccati equation
end
eta = 0.1;
N = 3;  % MPC Horizon

for s = 1:m
    x_mega{s} = sdpvar(repmat(nx,1,N+1), repmat(nd,1,N+1));  % x_mega{drone number}{scenario}{horizon step}
    u_mega{s} = sdpvar(repmat(nu,1,N), repmat(nd,1,N));
end

for d = 1:nd
    constraints{d} = [];
    objective{d} = 0;
end

for d = 1:nd
    for s = 1:m
        for k = 1:N
            % System dynamics constraints
            constraints{d} = [constraints{d}, x_mega{s}{k+1}(:,d) == A{s}*x_mega{s}{k}(:,d) + B{s}*u_mega{s}{k}(:,d)];
            constraints{d} = [constraints{d}, -a_lim*ones(nu,1) <= u_mega{s}{k}(:,d) <= a_lim*ones(nu,1)];
            % Objective function
            objective{d} = objective{d} + (x_mega{s}{k}(:,d)-targets(d,:)')'*Q*(x_mega{s}{k}(:,d)-targets(d,:)') + u_mega{s}{k}(:,d)'*R*u_mega{s}{k}(:,d);
        end
        % Terminal state cost
        objective{d} = objective{d} + eta*(x_mega{s}{N+1}(:,d)-targets(d,:)')'*QN{s}*(x_mega{s}{N+1}(:,d)-targets(d,:)');
    end
end

%% CBF data
combinations = nchoosek(1:nd, 2);
[lx, ly] = size(combinations);
% for c = 1:lx
%     for s = 1:m
%         for k = 1:N
%             % CBF constraints 
%             hk = abs(x_mega{s}{k}(1,combinations(c,1))-x_mega{s}{k}(1,combinations(c,2)))/r1 + abs(x_mega{s}{k}(2,combinations(c,1))-x_mega{s}{k}(2,combinations(c,2)))/r2 - 1;
%             hk1 =  abs(x_mega{s}{k+1}(1,combinations(c,1))-x_mega{s}{k+1}(1,combinations(c,2)))/r1 + abs(x_mega{s}{k+1}(2,combinations(c,1))-x_mega{s}{k+1}(2,combinations(c,2)))/r2 - 1;
%             constraints{combinations(c,1)} = [constraints{combinations(c,1)}, 0 <= hk1-hk+gamma*hk];
%             constraints{combinations(c,2)} = [constraints{combinations(c,2)}, 0 <= hk1-hk+gamma*hk];
%         end
%         hk0 = abs(x_mega{s}{1}(1,combinations(c,1))-x_mega{s}{1}(1,combinations(c,2)))/r1 + abs(x_mega{s}{1}(2,combinations(c,1))-x_mega{s}{1}(2,combinations(c,2)))/r2 - 1;
%         constraints{combinations(c,1)} = [constraints{combinations(c,1)}, 0 <= hk0];
%         constraints{combinations(c,2)} = [constraints{combinations(c,2)}, 0 <= hk0];
%     end
% end

for d = 1:nd
    U = [];
    for n = 1:N
        U = [U; u_mega{1}{n}(:,d)];
    end
    controller{d} = optimizer(constraints{d}, objective{d}, [], x_mega{1}{1}(:,d), U);
end

%% Simulation
for t = 1:T
    for d = 1:nd
        % Compute the optimal control action and check for infeasibility
        U = controller{d}(X(:,d,t));
        
        % Apply the control action
        X(:,d,t+1) = A0*X(:,d,t) + B0*U(1:nu);
        
        % Print the control action and time step
        fprintf('Control action u: [%s], for drone %d at time step %d\n', num2str(U'), d, t);
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
    plot(x_positions, y_positions, '-', 'LineWidth', 3, 'DisplayName', ['Drone ' num2str(drone_idx)]);
end

% Plot initial and final positions with distinct markers
for drone_idx = 1:nd
    % Extract the initial and final positions
    initial_x = X(1, drone_idx, 1);
    initial_y = X(2, drone_idx, 1);
    final_x = X(1, drone_idx, T+1);
    final_y = X(2, drone_idx, T+1);
    
    % Plot the initial position
    plot(initial_x, initial_y, 's', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', ['Initial Position Drone ' num2str(drone_idx)]);
    
    % Plot the final position
    plot(final_x, final_y, 'd', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', ['Final Position Drone ' num2str(drone_idx)]);
end

% Add labels and legend
xlabel('X Position');
ylabel('Y Position');
title('Trajectories of Drones');
legend('show');
grid on;
hold off;
