% Initialize ROS
rosinit;

% Initialize variables (as in your provided code)
h = 0.1;  % Sample time
A0 = [1 0 h 0; 0 1 0 h; 0 0 1 0; 0 0 0 1];
B0 = [h^2/2 0; 0 h^2/2; h 0; 0 h];
nx = 4;  % Number of states
nu = 2;  % Number of inputs
nd = 2;  % Number of drones

m = 119;  % Number of scenarios
r1 = 0.5;  % Drone proximity limits
r2 = 0.5;
gamma = 0.2;
a_lim = 1;  % Acceleration limit m/s^2

% Target destinations
targets = [3 0 0 0; -3 0 0 0]';
Q = 5 * eye(nx);
R = eye(nu);
eta = 0.1;
N = 5;  % MPC Horizon

mu = 0;
sigma = 1;
rng(1234);  % Setting the seed

for j = 1:nd
    Bd{j} = 5/1000*(-1)^(j+1)*[eye(2); zeros(2, 2)];
end

for s = 1:m
    for d = 1:nd
        for n = 1:N
            dis(:, s, n, d) = mu + sigma * randn(2, 1);
        end
    end
end

% Getting the maximum from all scenarios 
for d = 1:nd
    for n = 1:N
        disturbance(1, n, d) = max(dis(1, :, n, d));
        disturbance(2, n, d) = max(dis(2, :, n, d));
    end
end

QN = idare(A0, B0, Q, R, [], []);

% ROS Publishers
cmd_accel_x_pub1 = rospublisher('/robot1/accel_x', 'std_msgs/Float32');
cmd_accel_y_pub1 = rospublisher('/robot1/accel_y', 'std_msgs/Float32');
cmd_accel_x_pub2 = rospublisher('/robot2/accel_x', 'std_msgs/Float32');
cmd_accel_y_pub2 = rospublisher('/robot2/accel_y', 'std_msgs/Float32');

% ROS Subscribers
odom_sub1 = rossubscriber('/robot1/odom', 'nav_msgs/Odometry', @odom_callback1);
% odom_sub2 = rossubscriber('/robot2/odom', 'nav_msgs/Odometry', @odom_callback2);

% State Variables
global state1 state2;
state1 = zeros(4, 1);  % [pos_x; pos_y; vel_x; vel_y] for robot 1
state2 = zeros(4, 1);  % [pos_x; pos_y; vel_x; vel_y] for robot 2

state2 = [20;0;0;0];

% Threshold for stopping condition
threshold = 0.1;  % Distance to target

% Main loop to keep running the program until robots reach the targets
while true
    % Calculate control inputs
    tic
    U1 = DI_controller(state1, state2, N, A0, B0, Q, R, QN, r1, r2, gamma, eta, a_lim, Bd{1}, disturbance(:, :, 1), targets(:, 1))
    % U2 = DI_controller(state2, state1, N, A0, B0, Q, R, QN, r1, r2, gamma, eta, a_lim, Bd{2}, disturbance(:, :, 2), targets(:, 2));

    % Publish control inputs for robot 1
    msg_x1 = rosmessage(cmd_accel_x_pub1);
    msg_x1.Data = U1(1);
    send(cmd_accel_x_pub1, msg_x1);

    msg_y1 = rosmessage(cmd_accel_y_pub1);
    msg_y1.Data = U1(2);
    send(cmd_accel_y_pub1, msg_y1);

    % Publish control inputs for robot 2
    msg_x2 = rosmessage(cmd_accel_x_pub2);
    msg_x2.Data = U2(1);
    send(cmd_accel_x_pub2, msg_x2);

    msg_y2 = rosmessage(cmd_accel_y_pub2);
    msg_y2.Data = U2(2);
    send(cmd_accel_y_pub2, msg_y2);

    % Check if both robots have reached their targets
    dist1 = norm(state1(1:2) - targets(1:2, 1));
    dist2 = norm(state2(1:2) - targets(1:2, 2));

    if dist1 < threshold && dist2 < threshold
        disp('Both robots have reached their targets.');
        break;
    end

    % Wait until 0.3 seconds have passed since the start of the loop
    elapsed_time = toc;  % Get the elapsed time
    pause_time = 0.3 - elapsed_time;
    if pause_time > 0
        pause(pause_time);
    end
end

% Shutdown ROS
rosshutdown;

% Callback function for '/robot1/odom' topic
function odom_callback1(~, msg)
    global state1;
    % Update state1 based on the received odometry message
    state1(1) = msg.Pose.Pose.Position.X;
    state1(2) = msg.Pose.Pose.Position.Y;
    state1(3) = msg.Twist.Twist.Linear.X;
    state1(4) = msg.Twist.Twist.Linear.Y;
end

% Callback function for '/robot2/odom' topic
function odom_callback2(~, msg)
    global state2;
    % Update state2 based on the received odometry message
    state2(1) = msg.Pose.Pose.Position.X;
    state2(2) = msg.Pose.Pose.Position.Y;
    state2(3) = msg.Twist.Twist.Linear.X;
    state2(4) = msg.Twist.Twist.Linear.Y;
end
