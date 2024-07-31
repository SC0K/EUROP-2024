% Example Program for Bicycle Model with Forward and Backward Motion

% Simulation parameters
dt = 0.1; % Time step in seconds
T = 50; % Total number of time steps
L = 2.5; % Wheelbase of the vehicle in meters

% Initial state [x; y; theta; v]
initialState = [0; 0; 0; 0]; % Starting at origin, heading along x-axis, with 0 m/s speed

% Control inputs for acceleration and deceleration
a_x_forward = 1.0; % Acceleration in x direction (m/s^2)
a_y_forward = 0.0; % Acceleration in y direction (m/s^2)
a_x_backward = -1.0; % Deceleration in x direction (m/s^2)
a_y_backward = 1.0; % Deceleration in y direction (m/s^2)

% Initialize state history for plotting
stateHistory = zeros(4, T+1);
stateHistory(:, 1) = initialState;

% Simulation loop
for t = 1:T
    if t <= T/2
        % Accelerate forward in the first half of the simulation
        u = [a_x_forward; a_y_forward];
    else
        % Decelerate in the second half of the simulation
        u = [a_x_backward; a_y_backward];
    end
    
    % Predict the next state using the bicycle model
    [newState, ~, ~] = bicycleModelPredict(stateHistory(:, t), u, L, dt);
    
    % Store the new state
    stateHistory(:, t+1) = newState;
end

% Plotting the trajectory
figure;
hold on;
plot(stateHistory(1, :), stateHistory(2, :), 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Vehicle Trajectory using Bicycle Model');
grid on;
legend('Trajectory');
hold off;

%% Bicycle Model Prediction Function
function [newState, a_d, delta] = bicycleModelPredict(currentState, u, L, dt)
    % BICYCLEMODELPREDICT Predicts the next state using bicycle model dynamics
    % Inputs:
    %   currentState - Current state [x; y; theta; v]
    %   u - Control input [a_x; a_y]
    %   L - Wheelbase of the vehicle
    %   dt - Time step
    % Outputs:
    %   newState - Updated state [x; y; theta; v]
    %   a_d - Longitudinal acceleration
    %   delta - Steering angle

    % Extract current state
    x = currentState(1);
    y = currentState(2);
    theta = currentState(3);
    v = currentState(4);


    % Compute desired heading
    theta_d = atan2(u(2), u(1));
    
    % Compute desired longitudinal acceleration
    a_d = sqrt(u(1)^2 + u(2)^2);
    if dot([u(1), u(2)], [cos(theta), sin(theta)]) < 0
        a_d = -a_d; % Make acceleration negative if in the opposite direction of heading
    end

    % Compute steering angle
    if v == 0
        delta = 0;
    else
        delta = atan2(L * sin(theta_d - theta), v);
    end
    %% Alternative method    
%     syms acc del
%     eqns = [u(1) == (L*sqrt(0.25+1/tan(del)^2))/(L/tan(del))*acc*cos(theta+ atan(tan(del)/2)) + v^2/(L*sqrt(0.25+1/tan(del)^2))*cos(theta+ atan(tan(del)/2)+pi/2),
%         u(2) == (L*sqrt(0.25+1/tan(del)^2))/(L/tan(del))*acc*sin(theta+ atan(tan(del)/2)) + v^2/(L*sqrt(0.25+1/tan(del)^2))*sin(theta+ atan(tan(del)/2)+pi/2)];
%     S = solve(eqns,[acc, del]);
%     
%     delta = S.del
%     a_d = S.acc

    % Update state using the bicycle model dynamics
    newState = bicycleModelDynamics(currentState, a_d, delta, dt, L);
end

%% Bicycle Model Dynamics Function
function newState = bicycleModelDynamics(currentState, a, delta, dt, L)
    % BICYCLEMODELDYNAMICS Updates the vehicle state using the bicycle model dynamics
    % Inputs:
    %   currentState - Current state [x; y; theta; v]
    %   a - Longitudinal acceleration
    %   delta - Steering angle
    %   dt - Time step
    %   L - Wheelbase of the vehicle
    % Output:
    %   newState - Updated state [x; y; theta; v]
    
    % Extract current state
    x = currentState(1);
    y = currentState(2);
    theta = currentState(3);
    v = currentState(4);
    
    % Equations of motion
    x_dot = v * cos(theta);
    y_dot = v * sin(theta);
    theta_dot = (v / L) * tan(delta);
    %% Alternative method
%     phi = atan(tan(delta)/2);
%     rc = L / sqrt(1/4+ 1/tan(delta)^2);
%     r0 = L/tan(delta);
% 
%     % Equations of motion
%     x_dot = v * cos(theta+phi);
%     y_dot = v * sin(theta+phi);
%     
%     theta_dot = v / rc;
%     v_dot = a*rc/r0;
    
    %% Update state
    x = x + x_dot * dt;
    y = y + y_dot * dt;
    theta = theta + theta_dot * dt;
    v = v + v_dot * dt;
    
    % Ensure theta stays within -pi to pi
%     theta = mod(theta + pi, 2*pi) - pi;
    
    % Return the updated state
    newState = [x; y; theta; v];
end
