% Vehicle parameters
L = 2.5; % Wheelbase of the vehicle in meters

% Simulation parameters
dt = 0.1; % Time step in seconds
time = 0:dt:10; % 10 seconds of simulation time

% Initial state [x, y, theta, v]
state = [0, 0, 0, 0]; % Starting at origin, heading along x-axis, with 10 m/s speed

% Control inputs
a_x = 0.2; % Constant acceleration in x direction
a_y = 0.1; % Constant acceleration in y direction

% Lists to store the state history for plotting
x_history = zeros(length(time), 1);
y_history = zeros(length(time), 1);

% Simulation loop
for t = 1:length(time)
    % Update state using the convert and simulate function
    
a_x = sin(rand(1)*5-1) % Constant acceleration in x direction
a_y = sin(rand(1)*5-1) % Constant acceleration in y direction
    [state, a_d, delta] = convertAndSimulate(state, a_x, a_y, dt, L);
    
    % Store the updated state
    x_history(t) = state(1);
    y_history(t) = state(2);
end

% Plotting the trajectory
figure;
plot(x_history, y_history, 'DisplayName', 'Vehicle Path');
xlabel('x (m)');
ylabel('y (m)');
title('Bicycle Model Vehicle Trajectory');
legend('show');
grid on;




function [newState, a_d, delta] = convertAndSimulate(currentState, a_x, a_y, dt, L)
    % CONVERTANDSIMULATE Converts accelerations to bicycle model inputs and updates state
    % Inputs:
    %   currentState - Current state [x, y, theta, v]
    %   a_x - Acceleration in x direction
    %   a_y - Acceleration in y direction
    %   dt - Time step
    %   L - Wheelbase of the vehicle
    % Outputs:
    %   newState - Updated state [x, y, theta, v]
    %   a_d - Longitudinal acceleration
    %   delta - Steering angle
    
    % Extract current state
    x = currentState(1);
    y = currentState(2);
    theta = currentState(3);
    v = currentState(4);
    
    % Compute desired heading
    theta_d = atan2(a_y, a_x);
    
    % Compute desired longitudinal acceleration
    a_d = sqrt(a_x^2 + a_y^2);
    
    % Compute steering angle
    if v == 0
        delta = 0;
    else
        delta = atan2(L * sin(theta_d - theta), v);
    end
    
    % Update state using the bicycle model dynamics
    newState = bicycleModelDynamics(currentState, a_d, delta, dt, L);
end

function newState = bicycleModelDynamics(currentState, a, delta, dt, L)
    % BICYCLEMODELDYNAMICS Updates the vehicle state using the bicycle model dynamics
    % Inputs:
    %   currentState - Current state [x, y, theta, v]
    %   a - Longitudinal acceleration
    %   delta - Steering angle
    %   dt - Time step
    %   L - Wheelbase of the vehicle
    % Output:
    %   newState - Updated state [x, y, theta, v]
    
    % Extract current state
    x = currentState(1);
    y = currentState(2);
    theta = currentState(3);
    v = currentState(4);
    
    % Equations of motion
    x_dot = v * cos(theta);
    y_dot = v * sin(theta);
    theta_dot = (v / L) * tan(delta);
    v_dot = a;
    
    % Update state
    x = x + x_dot * dt;
    y = y + y_dot * dt;
    theta = theta + theta_dot * dt;
    v = v + v_dot * dt;
    
    % Return the updated state
    newState = [x, y, theta, v];
end
