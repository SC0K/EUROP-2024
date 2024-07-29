function [controllerState] = bicycleToControllerState(bicycleState)
    % BICYCLETOCONTROLLERSTATE Converts bicycle model states to controller states
    % Inputs:
    %   bicycleState - Current state [x, y, theta, v]
    % Outputs:
    %   controllerState - Converted state [p_x, p_y, v_x, v_y]
    
    % Extract bicycle model states
    x = bicycleState(1);
    y = bicycleState(2);
    theta = bicycleState(3);
    v = bicycleState(4);
    
    % Convert bicycle model states to controller states
    p_x = x;
    p_y = y;
    v_x = v * cos(theta);
    v_y = v * sin(theta);
    
    % Create controller state
    controllerState = [p_x; p_y; v_x; v_y];
end