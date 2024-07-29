% Version 6 - adding disturbance

yalmip('clear')
clear all

h = 0.2;       % Simulation sample time (s)
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
nd = 2; % Number of drones
T = 300; % Number of time steps

for j = 1:nd
    Bd{j} = 5/1000*(-1)^(j+1)*[eye(2);zeros(2,2)];
end

rng(1234); % Setting the seed

% Define the mean and standard deviation for disturbance
mu = 0;
sigma = 1; 

m = 1; % Number of scenarios
r1 = 0.2;     % Drone proximity limits
r2 = 0.2;
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

% d = ;

combinations = zeros(nd, nd-1);
for i = 1:nd
    temp = 1:nd;         % Create a temporary array with all drone indices
    temp(i) = [];        % Remove the i-th drone
    combinations(i, :) = temp;  % Store the result
end
[lx,ly] = size(combinations);


%% MPC data 
Q = 5*eye(nx);
R = 2*eye(nu);
eta = 0.1;
N = 3;  % MPC Horizon

for t = 1:T
    for ud = 1:nd
                u{ud} = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    end

    
    for d = 1:nd
        X_mpc = X(:,d,t);       % Getting the current state
        for s = 1:m
                QN = idare(A{s}, B{s}, Q, R, [], []);
                
                constraints = [];
                objective = 0;
%% MPC and CBF
            for c = 1:ly
                hk = abs(X_mpc(1,1) - X(1,combinations(d,c),t))/r1 + abs(X_mpc(2,1) - X(2,combinations(d,c),t))/r2 - 1;
%                 X_mpc1 = A{s}*X_mpc+B{s}*u{d}{1};
%                 hk1 = abs(X_mpc1(1,1) - X(1,combinations(d,c),t))/r1 + abs(X_mpc1(2,1) - X(2,combinations(d,c),t))/r2 - 1;     % Barrier function at k+1
%                 constraints = [constraints, 0 <= hk1-hk+gamma*hk];
                delta_vx = X_mpc(3,1) - X(3,combinations(d,c),t);
                delta_vy = X_mpc(4,1) - X(4,combinations(d,c),t);
                Gamma = 2/h^2*gamma*hk+ (delta_vx/r1 + delta_vy/r2)*2/h;
                vc = [1/r1, 1/r2];
                constraints = [constraints, [vc, vc]*[u{d}{1}; u{combinations(d,c)}{1}] - Gamma <= 0];
                
                for k = 1:N
                        
%                     X_mpc = A{s}*X_mpc+B{s}*u{d}{1}+Bd{d}*( mu + sigma * randn(2, 1));
                    X_mpc = A{s}*X_mpc+B{s}*u{d}{1};
%                     constraints = [constraints, -a_lim <= u{d}{k} <= a_lim];
                    constraints = [constraints, -a_lim*ones(nu,1) <= u{d}{k}, u{d}{k} <= a_lim*ones(nu,1)];
                    objective = objective + (X_mpc-targets(:,d))'*Q*(X_mpc-targets(:,d)) + u{d}{k}'*R*u{d}{k};
                    
                end
                objective = objective + eta*(X_mpc-targets(:,d))'*QN*(X_mpc-targets(:,d));
            end
        
        
        end
%% Simulation
options = sdpsettings( 'verbose', 1,'cache',-1);
diagnostics = optimize(constraints,objective,options);
if diagnostics.problem == 0
 disp('Solver thinks it is feasible')
elseif diagnostics.problem == 1
 disp('Solver thinks it is infeasible')
else
 disp('Something else happened')
end

U = value(u{d}(:,1));
X(:,d,t+1) = A0*X(:,d,t)+B0*U{1}+Bd{d}*( mu + sigma * randn(2, 1));
t
    end
end







%% Plotting Trajectories
figure;
hold on;

% Plot the trajectory of each drone with lines
for drone_idx = 1:nd
    % Extract the x and y positions of the drone over time
    x_positions = squeeze(X(1,  drone_idx, :));  % x positions
    y_positions = squeeze(X(2,  drone_idx, :));  % y positions
    
    % Plot the trajectory with lines
    plot(x_positions, y_positions, '-', 'LineWidth', 3,'DisplayName', ['Drone ' num2str(drone_idx)]);
end

% Plot initial and final positions with distinct markers
for drone_idx = 1:nd
    % Extract the initial and final positions
    initial_x = X(1,  drone_idx, 1);
    initial_y = X(2,  drone_idx, 1);
    final_x = X(1,  drone_idx, T+1);
    final_y = X(2,  drone_idx, T+1);
    
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


%% Plotting Trajectories with Animation
figure;
set(gcf, 'Position', [100, 100, 800, 800]); % [left, bottom, width, height]
hold on;

% Set up plot for animation
h_drones = gobjects(nd, 1);
for drone_idx = 1:nd
    h_drones(drone_idx) = plot(X(1, drone_idx, 1), X(2, drone_idx, 1), 'o-', 'LineWidth', 2, 'DisplayName', ['Drone ' num2str(drone_idx)]);
end

% Plot initial and final positions with distinct markers
for drone_idx = 1:nd
    % Extract the initial and final positions
    initial_x = X(1,  drone_idx, 1);
    initial_y = X(2,  drone_idx, 1);
    final_x = X(1,  drone_idx, T+1);
    final_y = X(2,  drone_idx, T+1);
    
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

% Create a video writer object
video_filename = 'drones_trajectory.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
open(v);

% Animation loop
for t = 1:T+1
    for drone_idx = 1:nd
        % Update drone positions
        set(h_drones(drone_idx), 'XData', squeeze(X(1, drone_idx, 1:t)));
        set(h_drones(drone_idx), 'YData', squeeze(X(2, drone_idx, 1:t)));
    end
    
    % Capture the plot as a frame
    frame = getframe(gcf);
    writeVideo(v, frame);
    
    pause(0.01); % Adjust the pause value to control the speed of the animation
end

% Close the video writer object
close(v);

hold off;