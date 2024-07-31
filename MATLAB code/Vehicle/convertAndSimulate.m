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
    
    % Compute steering angle
    if v == 0
        delta = 0;
    else
        delta = atan2(L * sin(theta_d - theta), v);
    end

    % Compute desired longitudinal acceleration
    a_d = sqrt(a_x^2 + a_y^2);
    if dot([a_x, a_y], [cos(theta), sin(theta)]) < 0
        a_d = -a_d; % Make acceleration negative if in the opposite direction of heading
    end

%     if abs(theta_d - theta) > deg2rad(90)
%         a_d = -sqrt(a_x^2 + a_y^2);
%     else
%         a_d = sqrt(a_x^2 + a_y^2);
%     end

    % Update state using the bicycle model dynamics
    newState = bicycleModelDynamics(currentState, a_d, delta, dt, L);
end
