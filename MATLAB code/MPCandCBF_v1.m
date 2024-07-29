% First version - only the MPC working. Need to modify the state
% structures: putting states of all drones in a single matrix.


yalmip('clear')
clear all

h = 0.1;       % Simulation sample time (s)
A0 = [1 0 h 0;
    0 1 0 h;
    0 0 1 0;
    0 0 0 1;];
B0 = [h^2/2 0;
    0 h^2/2;
    h 0;
    0 h;];
nx = 4; % Number of states
nu = 2; % Number of inputs
nd = 4; % Number of drones
T = 100; % Number of time steps

m = 1; % Number of scenarios
r1 = 0.25;     % Drone proximity limits
r2 = 0.25;
gamma = 0.2;
a_lim = 4;  % acceleration limit m/s^2
% 4 Drones
X = zeros(nx,1, 4,T+1);  % MegaState matrix (current states, 1 column, drone index, Time step)

for s = 1:m
    A{s} = A0;
    B{s} = B0;
end

% Initialize states
X(:,:,1,1) = [0;1;0;0];
X(:,:,2,1) = [0;-1;0;0];
X(:,:,3,1) = [1;0;0;0];
X(:,:,4,1) = [-1;0;0;0];

%% MPC data 
Q = 5*eye(nx);
R = 2*eye(nu);
for s = 1:m
    QN{s} = idare(A{s},B{s},Q,R,[],[]);  % Solution of Riccatic euqation
end
eta = 0.1;
N=3;    % MPC Horizon
      


x=sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
u=sdpvar(repmat(nu,1,N),repmat(1,1,N));

for s = 1:m
    for d= 1:nd
        x_mega{d}{s} = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));     % x_mega{drone number}{scenario}{horizon step}
        u_mega{d}{s} = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    end
end

for d = 1:nd
constraints{d} = [];
objective{d} = 0;
end

for d = 1:nd
    for s = 1:m
        for k = 1:N-1
             constraints{d} = [constraints{d}, x_mega{d}{s}{k+1} == A{s}*x_mega{d}{s}{k} + B{s}*u_mega{d}{s}{k}];
             constraints{d} = [constraints{d}, -a_lim*ones(nu,1) <= u_mega{d}{s}{k} <= a_lim*ones(nu,1)];
             % Target destination
             if even(d)
                objective{d} = objective{d} + (x_mega{d}{s}{k}-X(:,:,d-1,1))'*Q*(x_mega{d}{s}{k}-X(:,:,d-1,1))+ u_mega{d}{s}{k}'*R*u_mega{d}{s}{k};
             else
                 objective{d} = objective{d} + (x_mega{d}{s}{k}-X(:,:,d+1,1))'*Q*(x_mega{d}{s}{k}-X(:,:,d+1,1))+ u_mega{d}{s}{k}'*R*u_mega{d}{s}{k};
             end
        end
        if even(d)
            objective{d} = objective{d} + eta*(x_mega{d}{s}{N}-X(:,:,d-1,1))'*QN{s}*(x_mega{d}{s}{N}-X(:,:,d-1,1));
        else
            objective{d} = objective{d} + eta*(x_mega{d}{s}{N}-X(:,:,d+1,1))'*QN{s}*(x_mega{d}{s}{N}-X(:,:,d+1,1));    % Termination state
        end
    end
end

%% CBF data


combinations = nchoosek(1:nd, 2);
[lx,ly] = size(combinations);
for c = 1:lx
    for s = 1:m
        for k = 1: N-1
            % CBF constraints 
            hk = abs(x_mega{combinations(c,1)}{s}{k}(1)-x_mega{combinations(c,2)}{s}{k}(1))/r1 + abs(x_mega{combinations(c,1)}{s}{k}(2)-x_mega{combinations(c,2)}{s}{k}(2))/r2 - 2;
            hk1 = abs(x_mega{combinations(c,1)}{s}{k+1}(1)-x_mega{combinations(c,2)}{s}{k+1}(1))/r1 + abs(x_mega{combinations(c,1)}{s}{k+1}(2)-x_mega{combinations(c,2)}{s}{k+1}(2))/r2 - 2;
            constraints{combinations(c,1)} = [constraints{combinations(c,1)}, 0 <= hk1-hk+gamma*hk];
            constraints{combinations(c,2)} = [constraints{combinations(c,2)}, 0 <= hk1-hk+gamma*hk];
        end
            hk0 = abs(x_mega{combinations(c,1)}{s}{1}(1)-x_mega{combinations(c,2)}{s}{1}(1))/r1 + abs(x_mega{combinations(c,1)}{s}{1}(2)-x_mega{combinations(c,2)}{s}{1}(2))/r2 - 1;
            constraints{combinations(c,1)} = [constraints{combinations(c,1)}, 0 <= hk0];
            constraints{combinations(c,2)} = [constraints{combinations(c,2)}, 0 <= hk0];
    end
end

for d=1:nd
    controller{d} = optimizer(constraints{d}, objective{d},[],x_mega{d}{1}{1},[u_mega{d}{1}{:}]); %%% TO DO: states of all drones should be in one sdpvar group instead of cells 
end

%% Simulation
for t = 1:T+1
    for d = 1:nd
        [U, diagnostics] = controller{d}(X(:,:,d,t));
    
%          if diagnostics.problem ~= 0
%             error('The optimization problem is infeasible for drone %d at time step %d.\n', d, t);
%          end

    X(:,:,d,t+1) = A0*X(:,:,d,t)+B0*U(:,1);
    end
end



% % Create a new figure
% figure;
% hold on;
% 
% % Plot the trajectory of each drone
% for drone_idx = 1:nd
%     % Extract the x and y positions of the drone over time
%     x_positions = squeeze(X(1, 1, drone_idx, :));  % x positions
%     y_positions = squeeze(X(2, 1, drone_idx, :));  % y positions
%     
%     % Plot the trajectory
%     plot(x_positions, y_positions, 'DisplayName', ['Drone ' num2str(drone_idx)]);
% end
% 
% % Add labels and legend
% xlabel('X Position');
% ylabel('Y Position');
% title('Trajectories of Drones');
% legend('show');
% grid on;
% hold off;



figure;
hold on;

% Plot the trajectory of each drone with lines
for drone_idx = 1:nd
    % Extract the x and y positions of the drone over time
    x_positions = squeeze(X(1, 1, drone_idx, :));  % x positions
    y_positions = squeeze(X(2, 1, drone_idx, :));  % y positions
    
    % Plot the trajectory with lines
    plot(x_positions, y_positions, '-', 'LineWidth', 3,'DisplayName', ['Drone ' num2str(drone_idx)]);
end

% Plot initial and final positions with distinct markers
for drone_idx = 1:nd
    % Extract the initial and final positions
    initial_x = X(1, 1, drone_idx, 1);
    initial_y = X(2, 1, drone_idx, 1);
    final_x = X(1, 1, drone_idx, T+1);
    final_y = X(2, 1, drone_idx, T+1);
    
    % Plot the initial position
    plot(initial_x, initial_y, 's', 'MarkerSize', 10, 'DisplayName', ['Initial Position Drone ' num2str(drone_idx)]);
    
    % Plot the final position
    plot(final_x, final_y, 'd', 'MarkerSize', 10, 'DisplayName', ['Final Position Drone ' num2str(drone_idx)]);
end

% Add labels and legend
xlabel('X Position');
ylabel('Y Position');
title('Trajectories of Drones');
legend('show');
grid on;
hold off;