% function newState = bicycleModelDynamics(currentState, a, delta, dt, L)
%     % BICYCLEMODELDYNAMICS Updates the vehicle state using the bicycle model dynamics
%     % Inputs:
%     %   currentState - Current state [x; y; theta; v]
%     %   a - Longitudinal acceleration
%     %   delta - Steering angle
%     %   dt - Time step
%     %   L - Wheelbase of the vehicle
%     % Output:
%     %   newState - Updated state [x; y; theta; v]
%     
%     % Extract current state
%     x = currentState(1);
%     y = currentState(2);
%     theta = currentState(3);
%     v = currentState(4);
%     
%     % Equations of motion
%     x_dot = v * cos(theta);
%     y_dot = v * sin(theta);
%     theta_dot = (v / L) * tan(delta);
%     v_dot = a;
%     
%     % Update state
%     x = x + x_dot * dt;
%     y = y + y_dot * dt;
%     theta = theta + theta_dot * dt;
%     v = v + v_dot * dt;
%     
%     % Ensure theta stays within -pi to pi
%     theta = mod(theta + pi, 2*pi) - pi;
%     
%     % Return the updated state
%     newState = [x; y; theta; v];
% end












% No negative accel
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

