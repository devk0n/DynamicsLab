clear; close all; clc; format shortG;

% Constants
g = 9.81;               % Gravity
dt = 0.01;              % Time step
t_total = 10;           % Total simulation time
steps = t_total / dt;   % Number of time steps

% Initial Conditions
q1 = [1; 0; 0; 1; 0; 0; 0]; % Initial position and orientation
q1d = [0; 0; 0; 0; 0; 0; 0]; % Initial velocity
r1 = q1(1:3);
p1 = q1(4:7);

% Parameters
N1 = diag([10, 10, 10]);
J1m = diag([0.1, 0.1, 3]);

% Store results for visualization
time = (0:dt:t_total)';
positions = zeros(length(time), 3);
orientations = zeros(length(time), 4);

for step = 1:length(time)
    % Extract position and orientation components
    r1 = q1(1:3);
    p1 = q1(4:7);
    r1d = q1d(1:3);
    p1d = q1d(4:7);
    
    % L1 Matrix
    L1 = [-p1(2),  p1(1),  p1(4), -p1(3);
          -p1(3), -p1(4),  p1(1),  p1(2);
          -p1(4),  p1(3), -p1(2),  p1(1)];
    
    % Generalized Inertia Matrix
    J1s = 4 * L1' * J1m * L1;
    Ms = blkdiag(N1, J1s);
    
    % Constraint Matrix
    P = [zeros(1, 3), p1'];
    A = [Ms, P'; P, zeros(1, 1)];
    
    % Time Derivative of L1
    L1d = [-p1d(2),  p1d(1),  p1d(4), -p1d(3);
           -p1d(3), -p1d(4),  p1d(1),  p1d(2);
           -p1d(4),  p1d(3), -p1d(2),  p1d(1)];
    
    % Skew-Symmetric H1
    H1 = 4 * L1d' * J1m * L1;
    bs = [zeros(3, 1); 2 * H1 * p1d];
    
    % Additional Constraint
    c = p1d' * p1d;
    B = [bs; c];
    
    % External Forces and Torques
    f1 = [0; 0; -N1(1, 1) * g];
    n1s = [zeros(4, 1)];
    gs = [f1; n1s];
    c = [gs; 0];
    
    % Solve for accelerations and constraint force
    x = A \ (c - B);
    q1dd = x(1:7);  % Accelerations
    omega = x(8);   % Constraint force (not used in updates)
    
    % Update velocities and positions using Euler integration
    q1d = q1d + q1dd * dt; % Update velocity
    q1 = q1 + q1d * dt;    % Update position
    
    % Normalize the quaternion (p1 is the quaternion part of q1)
    p1 = q1(4:7);          % Extract quaternion
    q1(4:7) = p1 / norm(p1); % Normalize quaternion
    
    % Store results for visualization
    positions(step, :) = r1';
    orientations(step, :) = q1(4:7)';
end

% Visualization
figure;
subplot(2, 1, 1);
plot(time, positions);
xlabel('Time (s)');
ylabel('Position (m)');
legend('x', 'y', 'z');
title('Position over Time');

subplot(2, 1, 2);
plot(time, orientations);
xlabel('Time (s)');
ylabel('Orientation Components');
legend('p1', 'p2', 'p3', 'p4');
title('Orientation over Time');
