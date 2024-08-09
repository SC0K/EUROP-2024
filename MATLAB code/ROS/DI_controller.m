function U1 = DI_controller(state_main,state1,N, A0, B0, Q, R, QN, r1, r2, gamma,eta, a_lim, Bd, disturbances, target)
%DI_CONTROLLER Summary of this function goes here
    % input: state_main - state of robot that we calculate, state1 - state of the other robot, N - MPC
    % horizon.
    % Output: U1: accelerations control signal forthe main robot.
    u = sdpvar(repmat(2,1,N),repmat(1,1,N));
    constraints = [];
    objective = 0;
    for k = 1: N
        hk = (state_main(1) - state1(1))^2/r1^2 + (state_main(2)-state1(2))^2/r2^2 - 1;
        state_main_update = A0*state_main + B0* u{k} + Bd*disturbances(:,k);
        state_main_normal = A0*state_main + B0* u{k};
        objective = objective + (state_main_normal - target)'*Q*(state_main_normal-target) + u{k}'*R*u{k};
        hk1 = (state_main_update(1) - state1(1))^2/r1^2 + (state_main_update(2)-state1(2))^2/r2^2 - 1;
        constraints = [constraints, -a_lim <= u{k} <= a_lim, 0 <= hk1-hk+gamma*hk];
    end
    objective = objective + eta*(state_main_normal-target)'*QN*(state_main_normal-target);
    options = sdpsettings('verbose', 0,'cache', -1,'solver','ipopt');
    diagnostics = optimize(constraints,objective,options);
    if diagnostics.problem == 0
     disp('Solver thinks it is feasible')
    elseif diagnostics.problem == 1
     disp('Solver thinks it is infeasible')
    else
     disp('Something else happened')
    end
    U1 = value(u{1}(:,1));
end

