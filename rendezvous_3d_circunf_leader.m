% Init
clear;    % Clear variables
clc;      % Clear command window
close all;% Close all figures

% Timing
dt = 0.01;
Tmax = 15;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions
nAgents = 5;
nDim = 3;
xAxisL = 10;
yAxisL = 10;
zAxisL = 10;
X = [min([xAxisL, yAxisL, zAxisL]) * rand(1, nDim*nAgents)]';
X(1) = xAxisL/2;
X(2) = yAxisL/2;
X(3) = zAxisL*0.9;

% Define Dynamics
Adj = ones(nAgents, nAgents) - eye(nAgents);
leaderAdjWeight = 1;
Adj(:,1) = leaderAdjWeight * nAgents * ones(nAgents,1);
Din = -sum(Adj,2).*eye(nAgents);
Laplacian = Din + Adj;
% Leader dynamics: no update
Laplacian(1,:) = zeros(1, nAgents);
Laplacian = kron(Laplacian, eye(nDim));  % generalization to R^n

% State Space for Agents Positions
A = Laplacian;
B = eye(nAgents*nDim);
C = eye(nAgents*nDim);
D = zeros(nAgents*nDim);
sys = ss(A, B, C, D);

% Control Input: Only the leader is driven (agent 1)
u = zeros(1);
u = repmat(u, length(t0), nDim*nAgents);
for i = 1:length(t0)
    u(i,1) = xAxisL * cos(t0(i)*10); 
    u(i,2) = yAxisL * sin(t0(i)*10); 
    u(i,3) = zAxisL * sin(t0(i)*10);
end

% Simulate system
[Y, t] = lsim(sys, u, t0, X);

%% Plotting Setup
figure;
hold on;
grid on;
view(3);
xlabel("X");
ylabel("Y");
zlabel("Z");
title("3D Trajectories with Velocity Gradient Indication");

% Prepare distinct line styles (colors are now determined by speed)
lineStyles = {'-', '--', ':', '-.', '-'};

% Loop over each agent
for agnt = 1:nAgents
    idx = (agnt-1)*nDim + (1:nDim);
    traj = Y(:, idx);
    
    % Compute speed (magnitude of velocity) using finite differences
    v = zeros(size(traj));
    v(1,:) = (traj(2,:) - traj(1,:)) / dt;
    v(end,:) = (traj(end,:) - traj(end-1,:)) / dt;
    for j = 2:length(traj)-1
        v(j,:) = (traj(j+1,:) - traj(j-1,:)) / (2*dt);
    end
    speed = sqrt(sum(v.^2,2));
    
    % Create segments for continuous line with gradient:
    % Each segment is defined by two consecutive points.
    % We use the 'surface' command with FaceColor 'none' and EdgeColor 'interp'
    Xseg = [traj(1:end-1,1)'; traj(2:end,1)'];
    Yseg = [traj(1:end-1,2)'; traj(2:end,2)'];
    Zseg = [traj(1:end-1,3)'; traj(2:end,3)'];
    % Use the speed values at the beginning of each segment for color mapping.
    Cseg = [speed(1:end-1)'; speed(1:end-1)'];
    
    % Plot the segments with gradient along the trajectory
    if (agnt == 1)
        surface(Xseg, Yseg, Zseg, Cseg, 'FaceColor', 'none', ...
        'EdgeColor', 'interp', 'LineStyle', '-', 'LineWidth', 2);
    else
        surface(Xseg, Yseg, Zseg, Cseg, 'FaceColor', 'none', ...
            'EdgeColor', 'interp', 'LineStyle', ':', 'LineWidth', 2);
    end

    % Mark start and final positions
    if agnt == 1
        % Leader: Red Markers
        plot3(traj(1,1), traj(1,2), traj(1,3), 'ro', 'MarkerSize', 8, 'LineWidth', 2); % Start
        plot3(traj(end,1), traj(end,2), traj(end,3), 'rp', 'MarkerSize', 10, 'LineWidth', 2); % End
        
    else
        % Other Agents: Black Markers
        plot3(traj(1,1), traj(1,2), traj(1,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % Start
        plot3(traj(end,1), traj(end,2), traj(end,3), 'kp', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % End
    end
end

% Add a colorbar to indicate speed
colorbar;
colormap(jet); % Choose a colormap that suits your needs
%caxis([min(speed) max(speed)]); % Adjust limits based on speed values
hold off;
