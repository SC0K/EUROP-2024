% Version 9 - ADDING SCENARIOS TO quadraic CBF
% I think this is better than the lincear (absolute value) version

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
nd = 2; % Number of drones
T = 100; % Number of time steps

m = 119; % Number of scenarios
r1 = 0.5;     % Drone proximity limits
r2 = 0.5;
gamma = 0.2;
a_lim = 1;  % acceleration limit m/s^2

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




%% MPC data 
Q = 5*eye(nx);
R = 1*eye(nu);
eta = 0.1;
N = 5;  % MPC Horizon

%% Define the mean and standard deviation for disturbance
mu = 0;
sigma = 1; 
rng(1234); % Setting the seed

for j = 1:nd
    Bd{j} = 5/1000*(-1)^(j+1)*[eye(2);zeros(2,2)];
end

for s = 1:m
    for d = 1:nd
        for n = 1:N 
            dis(:,s,n,d) =  mu + sigma * randn(2, 1);
        end
    end
end

% Getting the maximum from all scenarios 
for d = 1:nd
        for n = 1:N
            disturbance(1,n,d) = max(dis(1,:,n,d));
            disturbance(2,n,d) = max(dis(2,:,n,d));
        end
end



combinations = zeros(nd, nd-1);
for i = 1:nd
    temp = 1:nd;         % Create a temporary array with all drone indices
    temp(i) = [];        % Remove the i-th drone
    combinations(i, :) = temp;  % Store the result
end
[lx,ly] = size(combinations);


%% Simulation - Main
for t = 1:T
    for d = 1:nd
        X_mpc = X(:,d,t);       % Getting the current state
        X_mpc_normal = X_mpc;
        
                QN = idare(A0, B0, Q, R, [], []);
                
                u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
                constraints = [];
                objective = 0;
%% MPC and CBF
            for c = 1:ly
                for k = 1:N
%                     hk0 = (X_mpc(1,1) - X(1,combinations(d,c),t))^2 + (X_mpc(2,1) - X(2,combinations(d,c),t))^2 - r1^2;
%                     constraints = [constraints, hk0 >= 0];  
                    hk = (X_mpc(1,1) - X(1,combinations(d,c),t))^2/r1^2 + (X_mpc(2,1) - X(2,combinations(d,c),t))^2/r2^2 - 1;     % Barrier function at k
                    X_mpc_normal = A{s}*X_mpc_normal+B{s}*u{k};
                    X_mpc = A0*X_mpc+B0*u{k}+Bd{d}*disturbance(:,k,d);
                    constraints = [constraints, -a_lim <= u{k} <= a_lim];
                    objective = objective + (X_mpc_normal-targets(:,d))'*Q*(X_mpc_normal-targets(:,d))+u{k}'*R*u{k};
                    hk1 = (X_mpc(1,1) - X(1,combinations(d,c),t))^2/r1^2 + (X_mpc(2,1) - X(2,combinations(d,c),t))^2/r2^2 - 1;     % Barrier function at k+1
                    constraints = [constraints, 0 <= hk1-hk+gamma*hk];
                end
                objective = objective + eta*(X_mpc_normal-targets(:,d))'*QN*(X_mpc_normal-targets(:,d));
            end
        
        
        
%% Simulation
options = sdpsettings('verbose', 0,'cache', -1,'solver','ipopt');
diagnostics = optimize(constraints,objective,options);
if diagnostics.problem == 0
 disp('Solver thinks it is feasible')
elseif diagnostics.problem == 1
 disp('Solver thinks it is infeasible')
else
 disp('Something else happened')
end

U = value(u(:,1));
U_rec{d}(:,t) = value(U{1});
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

%% 
%% Calculate the closest distance between drones
min_dist = inf;
distances = [];
for t = 1:T+1
    for i = 1:nd-1
        for j = i+1:nd
           
            dist = norm(X(1:2,i,t) - X(1:2,j,t),2);
            distances = [distances, dist];
            if dist < min_dist
                min_dist = dist
            end
          
        end
    end
end


% Plotting Histogram of Distances
figure;
histogram(distances, 'BinWidth', 0.05);
xlabel('Distance');
ylabel('Frequency');
title('Histogram of Distances Between Drones');
grid on;
xline(0.25, 'r', 'LineWidth', 2, 'Label', 'd = 0.25');

%% Plot accelerations
% figure 
% 
% for d = 1:nd
%     subplot(2,2,d)
%     hold on
%     plot(0.1:0.1:10,U_rec{d}(1,:),'LineWidth',2,'DisplayName', 'x acceleration')
%     plot(0.1:0.1:10,U_rec{d}(2,:),'LineWidth',2,'DisplayName', 'y acceleration')
%     xlabel('time (s)');
%     ylabel('Acceleration (m/s^{2})');
%     title(['Drone ' num2str(d)])
%     legend('show')
%     hold off
% end

figure 

% Initialize arrays to hold plot handles and labels
all_handles = [];
font_size = 13;
for d = 1:nd
    subplot(2, 2, d)
    hold on
    h1 = plot(0.1:0.1:10, U_rec{d}(1,:), 'LineWidth', 2, 'DisplayName', 'x acceleration');
    h2 = plot(0.1:0.1:10, U_rec{d}(2,:), 'LineWidth', 2, 'DisplayName', 'y acceleration');
    xlabel('time (s)', 'FontSize', font_size);
    ylabel('Acceleration (m/s^{2})', 'FontSize', font_size);
    title(['Drone ' num2str(d)], 'FontSize', font_size);
    xlim([0, 10]);
    ylim([-4, 4]);

    % Collect handles from the first subplot only
    if d == 2
        all_handles = [h1, h2];
    end
    
    hold off
end

% Create a common legend using the handles from the first subplot
legend(all_handles, {'x acceleration', 'y acceleration'},'Orientation', 'vertical', 'FontSize', font_size);



