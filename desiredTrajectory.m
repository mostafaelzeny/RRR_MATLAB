clear; clc; close all;

% Define robot parameters
L1 = 1; % Length of link 1
L2 = 1; % Length of link 2
m1 = 1; % Mass of link 1
m2 = 1; % Mass of link 2
I1 = 0.1; % Inertia of link 1
I2 = 0.1; % Inertia of link 2
g = 9.81; % Gravity

% Simulation parameters
dt = 0.01; % Time step
T = 10; % Total time
time = 0:dt:T; % Time vector

% Desired trajectory (circular path)
radius = 0.3;
omega = 1;
xd = radius * cos(omega * time) + 1;
yd = radius * sin(omega * time) + 1;


% Initialize state variables
theta1 = zeros(size(time));
theta2 = zeros(size(time));
dtheta1 = zeros(size(time));
dtheta2 = zeros(size(time));

% PD controller gains
Kp = 100;
Kd = 20;

% Initialize arrays for storing results
theta1_desired = zeros(size(time));
theta2_desired = zeros(size(time));
x_actual = zeros(size(time));
y_actual = zeros(size(time));

for i = 1:length(time)
    % Inverse Kinematics to get desired joint angles
    theta2_desired(i) = acos((xd(i)^2 + yd(i)^2 - L1^2 - L2^2) / (2 * L1 * L2));
    theta1_desired(i) = atan2(yd(i), xd(i)) - atan2(L2 * sin(theta2_desired(i)), L1 + L2 * cos(theta2_desired(i)));
    
    % Compute errors
    e_theta1 = theta1_desired(i) - theta1(i);
    e_theta2 = theta2_desired(i) - theta2(i);
    e_dtheta1 = 0 - dtheta1(i);
    e_dtheta2 = 0 - dtheta2(i);
    
    % PD controller
    tau1 = Kp * e_theta1 + Kd * e_dtheta1;
    tau2 = Kp * e_theta2 + Kd * e_dtheta2;
    
    % Robot dynamics (simplified model)
    d2theta1 = tau1 / I1;
    d2theta2 = tau2 / I2;
    
    % Update state using Euler integration
    dtheta1(i+1) = dtheta1(i) + d2theta1 * dt;
    dtheta2(i+1) = dtheta2(i) + d2theta2 * dt;
    theta1(i+1) = theta1(i) + dtheta1(i) * dt;
    theta2(i+1) = theta2(i) + dtheta2(i) * dt;
    
    % Forward Kinematics to get actual end-effector position
    x_actual(i) = L1 * cos(theta1(i)) + L2 * cos(theta1(i) + theta2(i));
    y_actual(i) = L1 * sin(theta1(i)) + L2 * sin(theta1(i) + theta2(i));
end

% Plot desired and actual trajectories
figure;
plot(xd, yd, 'b', 'LineWidth', 2); hold on;
plot(x_actual, y_actual, 'r--', 'LineWidth', 2);
legend('Desired Trajectory', 'Actual Trajectory');
xlabel('X Position');
ylabel('Y Position');
title('End-Effector Trajectory');
grid on;

% Animation of the robot arm
figure;
for i = 1:10:length(time)
    clf;
    
    % Calculate link positions
    x1 = L1 * cos(theta1(i));
    y1 = L1 * sin(theta1(i));
    x2 = x1 + L2 * cos(theta1(i) + theta2(i));
    y2 = y1 + L2 * sin(theta1(i) + theta2(i));
    
    % Plot the links
    plot([0 x1], [0 y1], 'r', 'LineWidth', 2); hold on;
    plot([x1 x2], [y1 y2], 'b', 'LineWidth', 2);
    
    % Plot joints
    plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot(x1, y1, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot(x2, y2, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    
    % Plot end-effector trajectory
    plot(x_actual(1:i), y_actual(1:i), 'g--', 'LineWidth', 1.5);
    
    axis equal;
    axis([-1 2 -1 2]);
    grid on;
    drawnow;
end
