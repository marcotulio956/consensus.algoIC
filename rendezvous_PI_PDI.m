%% Comparison: Single Integrator vs. Double Integrator Dynamics
clear; clc; close all;

%% Timing and dimensions
dt = 0.01; Tmax = 5;
t0 = 0:dt:Tmax;
nAgents = 4; nDim = 3;
xAxisL = 10; yAxisL = 10; zAxisL = 10;

%% Define Adjacency and Laplacian (leader’s dynamics off)
Adj = ones(nAgents) - eye(nAgents);
leaderAdjWeight = 1;
Adj(:,1) = leaderAdjWeight * nAgents * ones(nAgents,1);
Din = -sum(Adj,2).*eye(nAgents);
Laplacian = Din + Adj;
Laplacian(1,:) = zeros(1,nAgents);

%% 1. Single Integrator Dynamics (Consensus with Leader Injection)
% Dynamics: x_dot = u, with u = -k*(Laplacian kron I)*x + E*u_leader.
k = 1;
A_single = -k * kron(Laplacian, eye(nDim));
E_single = zeros(nAgents*nDim, nDim);
E_single(1:nDim,:) = eye(nDim);  % only leader gets input
sys_single = ss(A_single, E_single, eye(nAgents*nDim), zeros(nAgents*nDim, nDim));

% Initial condition (force leader’s position to zero)
X_single0 = min([xAxisL, yAxisL, zAxisL]) * rand(nAgents*nDim,1);
X_single0(1:nDim) = 0;

% Leader’s input (helical trajectory)
u_leader_single = zeros(length(t0), nDim);
for i = 1:length(t0)
    u_leader_single(i,1) = xAxisL/2*cos(t0(i)) - yAxisL/2*sin(t0(i));
    u_leader_single(i,2) = xAxisL/2*sin(t0(i)) + yAxisL/2*cos(t0(i));
    u_leader_single(i,3) = 0.8*zAxisL;
end

% Simulate single integrator system
[Y_single, ~] = lsim(sys_single, u_leader_single, t0, X_single0);

%% 2. Double Integrator Dynamics (Consensus with Leader Injection)
% State: [x; v] ∈ R^(2*nAgents*nDim).
% Dynamics: [x_dot; v_dot] = [0 I; -k_p*(L kron I) -k_d*(L kron I)]*[x; v] + E_double*u_leader.
k_p = 10; k_d = 5;
A_double = [zeros(nAgents*nDim) eye(nAgents*nDim);
            -k_p*kron(Laplacian, eye(nDim)) -k_d*kron(Laplacian, eye(nDim))];
% Inject leader acceleration in the second half (for agent 1)
E_double = zeros(2*nAgents*nDim, nDim);
E_double(nAgents*nDim+1:nAgents*nDim+nDim, :) = eye(nDim);
sys_double = ss(A_double, E_double, eye(2*nAgents*nDim), zeros(2*nAgents*nDim, nDim));

% Initial condition: random positions and velocities (leader’s position zero)
X_double0 = min([xAxisL, yAxisL, zAxisL]) * rand(2*nAgents*nDim,1);
X_double0(1:nDim) = 0;

% Leader’s acceleration input:
% For a helical trajectory x = (xAxisL/2*cos(t)-yAxisL/2*sin(t)), its acceleration is:
%   a_x = -xAxisL/2*cos(t)+yAxisL/2*sin(t)
%   a_y = -xAxisL/2*sin(t)-yAxisL/2*cos(t)
% For z we use a constant zero acceleration.
u_leader_double = zeros(length(t0), nDim);
for i = 1:length(t0)
    u_leader_double(i,1) = -xAxisL/2*cos(t0(i)) + yAxisL/2*sin(t0(i));
    u_leader_double(i,2) = -xAxisL/2*sin(t0(i)) - yAxisL/2*cos(t0(i));
    u_leader_double(i,3) = 0;
end

% Simulate double integrator system
[Y_double, ~] = lsim(sys_double, u_leader_double, t0, X_double0);

%% Plot trajectories (positions only)
figure;
subplot(1,2,1)
title('Single Integrator Trajectories')
hold on
for ag = 1:nDim:nAgents*nDim
    plot3(Y_single(:,ag), Y_single(:,ag+1), Y_single(:,ag+2), 'LineWidth',1.5)
end
legend('Leader','Agent 2','Agent 3','Agent 4')
xlabel('X'); ylabel('Y'); zlabel('Z'); grid on; view(3);
hold off

subplot(1,2,2)
title('Double Integrator Trajectories (Positions)')
% For double integrator, positions are the first nAgents*nDim entries.
hold on
for ag = 1:nDim:nAgents*nDim
    plot3(Y_double(:,ag), Y_double(:,ag+1), Y_double(:,ag+2), 'LineWidth',1.5)
end
legend('Leader','Agent 2','Agent 3','Agent 4')
xlabel('X'); ylabel('Y'); zlabel('Z'); grid on; view(3);
hold off

sgtitle('Consensus: Single vs. Double Integrator Dynamics');
