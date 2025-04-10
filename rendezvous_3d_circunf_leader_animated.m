% Init
clear % vars
clc   % screen
close all % figures

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
X = [min([xAxisL yAxisL zAxisL]) * rand(1,nDim*nAgents)]';
X(1) = xAxisL/2;
X(2) = yAxisL/2;
X(3) = zAxisL*0.9;

% Define Dynamics
Adj = ones(nAgents,nAgents) - eye(nAgents);
leaderAdjWeight = 1;
Adj(:,1) = leaderAdjWeight * nAgents * ones(nAgents,1);
Din = -sum(Adj,2) .* eye(nAgents);
Laplacian = Din + Adj;
% Leader dynamics: no update
Laplacian(1,:) = zeros(1,nAgents);
Laplacian = kron(Laplacian, eye(nDim)); % generalization to R^n

% State Space for Agents Positions
A = Laplacian;
B = eye(nAgents*nDim);
C = eye(nAgents*nDim); % output Y matrix
D = zeros(nAgents*nDim);
sys = ss(A,B,C,D);

% Control Input: Only the leader is driven
u = zeros(1);
u = repmat(u, length(t0), nDim*nAgents);
for i = 1:length(t0)
    u(i,1) = xAxisL * cos(t0(i)*10); 
    u(i,2) = yAxisL * sin(t0(i)*10); 
    u(i,3) = zAxisL * sin(t0(i)*10);
end

% Simulate system
[Y, t] = lsim(sys, u, t0, X);

%% Setup Animation
figure;
hold on;
grid on;
view(3);
xlabel("X_i")
ylabel("X_j")
zlabel("X_k")
title("Rendezvous with Random Positions and Circular Leader (n=5)") 
title("Animated 3D Trajectories of Agents")

% Prepare distinct colors and line styles for each agent
colors = lines(nAgents);        
lineStyles = {'-', '--', ':', '-.', '-'}; % lineStyles{mod(agnt-1, length(lineStyles))+1}

% Preallocate handles for trails and markers
hTraj = gobjects(nAgents,1);
hAgent = gobjects(nAgents,1);

% Initial plot for each agent: start with a point for the trail and current position marker
for agnt = 1:nAgents
    idx = (agnt-1)*nDim + (1:nDim);
    traj = Y(:, idx);
    
    if (agnt==1)
        % Plot the initial point for the trajectory
        hTraj(agnt) = plot3(traj(1,1), traj(1,2), traj(1,3), ...
            'Color', colors(agnt,:), 'LineStyle', '-', ...
            'LineWidth', 1.5);
    else
        hTraj(agnt) = plot3(traj(1,1), traj(1,2), traj(1,3), ...
            'Color', colors(agnt,:), 'LineStyle', '--', ...
            'LineWidth', 1.5);
    end
    
    % Plot the current position marker (circle)
    hAgent(agnt) = plot3(traj(1,1), traj(1,2), traj(1,3), ...
        'o', 'MarkerSize', 8, 'MarkerFaceColor', colors(agnt,:), 'Color', colors(agnt,:));
end

%% Animation Loop
for k = 2:length(t0)
    for agnt = 1:nAgents
        idx = (agnt-1)*nDim + (1:nDim);
        traj = Y(:, idx);
        
        % Update the trajectory plot to include points up to the current time step k
        set(hTraj(agnt), 'XData', traj(1:k,1), 'YData', traj(1:k,2), 'ZData', traj(1:k,3));
        
        % Update the current position marker to the current point
        set(hAgent(agnt), 'XData', traj(k,1), 'YData', traj(k,2), 'ZData', traj(k,3));
    end
    drawnow;
    pause(0.01); % Adjust pause time for smoothness/speed of animation
end

% Mark final positions with a star
for agnt = 1:nAgents
    idx = (agnt-1)*nDim + (1:nDim);
    traj = Y(:, idx);
    plot3(traj(end,1), traj(end,2), traj(end,3), 'p', 'MarkerSize', 10, ...
          'MarkerFaceColor', colors(agnt,:), 'Color', colors(agnt,:));
end

hold off;
