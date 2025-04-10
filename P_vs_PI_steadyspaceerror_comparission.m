%% P and PI Control Simulation with Distance Error Plots
% Init
clear; clc; close all;

% Timing
dt = 0.01;
Tmax = 5;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions (3D)
nAgents = 4;
nDim = 3;
xAxisL = 10;
yAxisL = 10;
zAxisL = 10;
X1 = [min([xAxisL yAxisL zAxisL]) .* rand(1, nDim*nAgents)]';
% Ensure leader starts at the origin:
X1(1:3) = 0;

% Define Adjacency for both dynamics
Adj = ones(nAgents) - eye(nAgents);
leaderAdjWeight = 1;
Adj(:,1) = leaderAdjWeight * nAgents * ones(nAgents,1);
Din = -sum(Adj,2) .* eye(nAgents);
Laplacian = Din + Adj;
Laplacian(1,:) = zeros(1,nAgents);  % Leader dynamics

%% P Control (Proportional Only)
Laplacian1 = kron(Laplacian, eye(nDim));  % extend to R^n
A1 = Laplacian1;
B1 = eye(nAgents*nDim);
C1 = eye(nAgents*nDim);
D1 = zeros(nAgents*nDim);
sys1 = ss(A1, B1, C1, D1);

% Control Input for P (Helical Leader trajectory)
u1 = zeros(length(t0), nDim*nAgents);
for i = 1:length(t0)
    u1(i,1) = xAxisL/2*cos(t0(i)) - yAxisL/2*sin(t0(i));
    u1(i,2) = xAxisL/2*sin(t0(i)) + yAxisL/2*cos(t0(i));
    u1(i,3) = 0.8*zAxisL;
end

% Simulate P Control
[Y, ~] = lsim(sys1, u1, t0, X1);

%% PI Control (Proportional-Integral)
nCtrl = 2; % [x, integral(x)]
Ki = 30; 
Kp = 10;
X2 = [min([xAxisL yAxisL zAxisL]) .* rand(1, nDim*nAgents*nCtrl)]';

% Build Laplacian2 for PI dynamics: Each agent’s state contains the position and its integral.
% Start with the basic block [Kp 0; 1 0] and then add the integral gain on the appropriate elements.
Laplacian2 = kron(Laplacian, [Kp 0; 1 0]);  
for i = 2:2:(size(Laplacian2,1)-1)
    Laplacian2(i+1, i+2) = Ki;  % add Ki on the integrator channel for each agent
end
Laplacian2 = kron(Laplacian2, eye(nDim));
A2 = Laplacian2;
B2 = eye(nAgents*nDim*nCtrl);
C2 = eye(nAgents*nDim*nCtrl);
D2 = zeros(nAgents*nDim*nCtrl);
sys2 = ss(A2, B2, C2, D2);

% Control Input for PI (Helical Leader trajectory)
u2 = zeros(length(t0), nDim*nAgents*nCtrl);
for i = 1:length(t0)
    u2(i,1) = xAxisL/2*cos(t0(i)) - yAxisL/2*sin(t0(i));
    u2(i,2) = xAxisL/2*sin(t0(i)) + yAxisL/2*cos(t0(i));
    u2(i,3) = 0.8*zAxisL;
    % Other channels (e.g. integral part) remain zero if not actuated directly.
end

% Simulate PI Control
[Y2, ~] = lsim(sys2, u2, t0, X2);

%% Plotting the agent trajectories (as before)
figure;
subplot(1,2,1)
title('P Control Trajectories')
hold on
for agnt = 1:nDim:nDim*nAgents
    plot3(Y(:,agnt), Y(:,agnt+1), Y(:,agnt+2), 'LineWidth', 1.5)
end
legend('Agent 1 (Leader)', 'Agent 2', 'Agent 3', 'Agent 4')
xlabel("X"); ylabel("Y"); zlabel("Z")
grid on; view(3);
hold off

subplot(1,2,2)
title('PI Control Trajectories')
hold on
for agnt = 1:nDim*nCtrl:nDim*nAgents*nCtrl
    plot3(Y2(:,agnt), Y2(:,agnt+1), Y2(:,agnt+2), 'LineWidth', 1.5)
end
legend('Agent 1 (Leader)', 'Agent 2', 'Agent 3', 'Agent 4')
xlabel("X"); ylabel("Y"); zlabel("Z")
grid on; view(3);
hold off

sgtitle('Rendezvous with Random Positions and Helical Leader (n=4)');

%% Compute and Plot Sum of Distance Errors (from leader)
% For P Control:
sumError_P = zeros(length(t0),1);
for k = 1:length(t0)
    leaderPos = Y(k, 1:3); % first agent's position
    errorSum = 0;
    for ag = 2:nAgents
        idx = (ag-1)*nDim + (1:nDim);
        errorSum = errorSum + norm(Y(k, idx) - leaderPos);
    end
    sumError_P(k) = errorSum;
end

% For PI Control:
sumError_PI = zeros(length(t0),1);
for k = 1:length(t0)
    leaderPos = Y2(k, 1:3);
    errorSum = 0;
    for ag = 2:nAgents
        idx = (ag-1)*nDim*nCtrl + (1:nDim);
        errorSum = errorSum + norm(Y2(k, idx) - leaderPos);
    end
    sumError_PI(k) = errorSum;
end

% Plot the error curves
figure;
subplot(1,2,1)
plot(t0, sumError_P, 'r', 'LineWidth', 2)
title('P Control')
xlabel('Time [s]'); ylabel('Sum of Errors'); grid on

subplot(1,2,2)
plot(t0, sumError_PI, 'g', 'LineWidth', 2)
title('PI Control')
xlabel('Time [s]'); ylabel('Sum of Errors'); grid on

sgtitle('Convergence of Distance Error (Leader-Followers)');
