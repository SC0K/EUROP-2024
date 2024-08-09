% Initialize ROS
rosinit;

% --- ROS Publishers ---

% Publisher for '/robot1/accel_x' topic
cmd_accel_x_pub1 = rospublisher('/robot1/accel_x', 'std_msgs/Float32');

% Publisher for '/robot1/accel_y' topic
cmd_accel_y_pub1 = rospublisher('/robot1/accel_y', 'std_msgs/Float32');

% Publisher for '/robot2/accel_x' topic
cmd_accel_x_pub2 = rospublisher('/robot2/accel_x', 'std_msgs/Float32');

% Publisher for '/robot2/accel_y' topic
cmd_accel_y_pub2 = rospublisher('/robot2/accel_y', 'std_msgs/Float32');

% --- ROS Subscribers ---

% Subscriber for '/robot1/odom' topic
odom_sub1 = rossubscriber('/robot1/odom', 'nav_msgs/Odometry', @odom_callback1);

% Subscriber for '/robot2/odom' topic
odom_sub2 = rossubscriber('/robot2/odom', 'nav_msgs/Odometry', @odom_callback2);

% --- Callback Functions ---
state1 = zeros(4, 1);  % [pos_x; pos_y; vel_x; vel_y] for robot 1
state2 = zeros(4, 1);  % [pos_x; pos_y; vel_x; vel_y] for robot 2

% Callback function for '/robot1/odom' topic
function odom_callback1(~, msg)
    % Process the odometry message for robot1
    global state1;
    % Update state1 based on the received odometry message
    state1(1) = msg.Pose.Pose.Position.X;
    state1(2) = msg.Pose.Pose.Position.Y;
    state1(3) = msg.Twist.Twist.Linear.X;
    state1(4) = msg.Twist.Twist.Linear.Y;

end

% Callback function for '/robot2/odom' topic
function odom_callback2(~, msg)
    % Access global state2 variable
    global state2;
    % Update state2 based on the received odometry message
    state2(1) = msg.Pose.Pose.Position.X;
    state2(2) = msg.Pose.Pose.Position.Y;
    state2(3) = msg.Twist.Twist.Linear.X;
    state2(4) = msg.Twist.Twist.Linear.Y;
end
