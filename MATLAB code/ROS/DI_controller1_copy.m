function [U1,U2,feas] = DI_controller1_copy(state_main,state_bot2,N, A0, B0, Q, R, QN, r1, r2, gamma,eta, a_lim, Bd, disturbances, target,r_roundabout,D)
%DI_CONTROLLER1 - Safety constraints to all inputs
    % input: state_main - state of robot that we calculate, state1 - state of the other robot, N - MPC
    % horizon, D - road width, r_roundabout - roundabout radius
    % Output: U1, U2, feas: first and second accelerations control signal forthe main robot, feasibility of solution.
    u = sdpvar(repmat(2,1,N),repmat(1,1,N));
    constraints = [];
    objective = 0;
    state_main_update = state_main;
    state_main_normal = state_main;
    for k = 1: N
        hk = (state_main_update(1) - state_bot2(1))^2/r1^2 + (state_main_update(2)-state_bot2(2))^2/r2^2 - 1;% Collision constraints between robots
        hkk = (state_main_update(1)^2 + state_main_update(2)^2)-  r_roundabout^2- (D/2)^2;  % Roundabout constraint
        state_main_update = A0*state_main_update + B0* u{k};
        state_main_normal = A0*state_main_normal + B0* u{k};
        objective = objective + (state_main_normal - target)'*Q*(state_main_normal-target) + u{k}'*R*u{k};
        hk1 = (state_main_update(1) - state_bot2(1))^2/r1^2 + (state_main_update(2)-state_bot2(2))^2/r2^2 - 1;
        hkk1 = (state_main_update(1)^2 + state_main_update(2)^2) - r_roundabout^2 - (D/2)^2;
        % constraints = [constraints, -a_lim <= u{k} <= a_lim, 0 <= hk1-hk+gamma*hk];
        constraints = [constraints, -a_lim <= u{k} <= a_lim];
        constraints = [constraints, -0.15<=state_main_update(3)<=0.15, -0.15<=state_main_update(4)<=0.15];  % Speed limits
        if abs(state_main(2))<0.2 && abs(state_main(1)) > r_roundabout + D/2
            constraints = [constraints, -0.3<=state_main_update(2)<=0.3];
        else
            constraints = [constraints, 0 <= hkk1 - hkk+ gamma*hkk];
        end
    end
    objective = objective + eta*(state_main_normal-target)'*QN*(state_main_normal-target);
    options = sdpsettings('verbose', 0,'cache', -1,'solver','ipopt');
    diagnostics = optimize(constraints,objective,options);
    if diagnostics.problem == 0
     disp('Solver thinks it is feasible')
     feas = 0;
    elseif diagnostics.problem == 1
     fprintf(2, 'Solver thinks it is infeasible\n');
     feas = 1;
    else
     disp('Something else happened')
     feas = 1;
    end
    U1 = value(u{1}(:,1));
    U2 = value(u{2}(:,1));
end

