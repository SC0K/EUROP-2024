% Initialize ROS
rosshutdown
rosinit;
clear all
yalmip('clear')
% Initialize variables (as in your provided code)
h = 0.4;  % Sample time
A0 = [1 0 h 0; 0 1 0 h; 0 0 1 0; 0 0 0 1];
B0 = [h^2/2 0; 0 h^2/2; h 0; 0 h];
nx = 4;  % Number of states
nu = 2;  % Number of inputs
nd = 2;  % Number of drones

m = 119;  % Number of scenarios
r1 = 0.7;  % Drone proximity limits
r2 = 0.7;
gamma = 0.2;
a_lim = 0.5;  % Acceleration limit m/s^2

% Target destinations
targets = [5.0 0 0 0; -5.0 0 0 0]';
Q = 5 * eye(nx);
R = eye(nu);
eta = 0.1;
N = 4;  % MPC Horizon

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

t_max=0;
n=0;
t_total=0;

% ROS Publishers
cmd_position_pub1 = rospublisher('/robot1/move_base_simple/goal', 'geometry_msgs/PoseStamped');
cmd_position_pub2 = rospublisher('/robot2/move_base_simple/goal', 'geometry_msgs/PoseStamped');

% ROS Subscribers
odom_sub1 = rossubscriber('/robot1/odom', 'nav_msgs/Odometry', @odom_callback1);
odom_sub2 = rossubscriber('/robot2/odom', 'nav_msgs/Odometry', @odom_callback2);

% State Variables
global state1 state2;
state1 = zeros(4, 1);  % [pos_x; pos_y; vel_x; vel_y] for robot 1
state2 = zeros(4, 1);  % [pos_x; pos_y; vel_x; vel_y] for robot 2

% Threshold for stopping condition
threshold = 0.1;  % Distance to target

% Main loop to keep running the program until robots reach the targets
while true
    % Calculate control inputs
    tic
    % [U1,U1_1] = DI_controller1(state1, state2, N, A0, B0, Q, R, QN, r1, r2, gamma, eta, a_lim, Bd{1}, disturbance(:, :, 1), targets(:, 1), 2.75, 1.1);
    [U2,U2_1] = DI_controller1(state2, state1, N, A0, B0, Q, R, QN, r1, r2, gamma, eta, a_lim, Bd{2}, disturbance(:, :, 2), targets(:, 2), 2.75, 1.1);
    if  all(~isnan(U2(:))) && all(~isnan(U2_1(:)))
        % P1 = A0*state1+B0*U1;
        P2 = A0*state2+B0*U2;
        % % Publish control inputs for robot 1
        % msg_p1 = rosmessage(cmd_position_pub1);
        % msg_p1.Pose.Position.X = P1(1);
        % msg_p1.Pose.Position.Y = P1(2);
        % send(cmd_position_pub1, msg_p1);
    
    
        % Publish control inputs for robot 2
        msg_p2 = rosmessage(cmd_position_pub2);
        msg_p2.Pose.Position.X = P2(1);
        msg_p2.Pose.Position.Y = P2(2);
        send(cmd_position_pub2, msg_p2);
    
        % Check if both robots have reached their targets
        dist1 = norm(state1(1:2) - targets(1:2, 1));
        dist2 = norm(state2(1:2) - targets(1:2, 2));

        if dist1 < threshold && dist2 < threshold
            disp('Both robots have reached their targets.');
            break;
        end
    
        % Wait until 0.4 seconds have passed since the start of the loop
        elapsed_time = toc;  % Get the elapsed time
        if elapsed_time>t_max
            t_max = elapsed_time;
        end
        n=n+1;
        t_total = elapsed_time + t_total;
        t_average = t_total/n;
        pause_time = 0.4 - elapsed_time;
        if pause_time > 0
            pause(pause_time);
        end
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
    v_forward = msg.Twist.Twist.Linear.X;
    % Example quaternion [w, x, y, z]
    quaternion = [msg.Pose.Pose.Orientation.W, msg.Pose.Pose.Orientation.X, msg.Pose.Pose.Orientation.Y, msg.Pose.Pose.Orientation.Z];
    
    % Convert quaternion to Euler angles (in degrees)
    euler_angles = quat2eul(quaternion);
    
    % Convert yaw to radians if needed
    yaw_rad = deg2rad(euler_angles(1));

    state1(3) = v_forward*sin(yaw_rad);
    state1(4) = v_forward*cos(yaw_rad);
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
