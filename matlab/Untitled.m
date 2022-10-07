% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt = 0.01;
Tmax = 10;
t = 0:dt:Tmax;
T = 0;

% Agents Data Positions 
nAgents = 1;
nDim = 3;
xAxisL = 10;
yAxisL = 10;
zAxisL = 10;
X = min([xAxisL yAxisL zAxisL]).*rand(nAgents*nDim,length(t));
X(:,1:nDim) = 0;

% Controls
u=zeros(nDim*nAgents,length(t));
u(1,:)=xAxisL/2*cos(t)-yAxisL/2*sin(t); % Leader
u(2,:)=xAxisL/2*sin(t)+yAxisL/2*cos(t);
u(3,:)=0.8*zAxisL;

% Sim
while (T<Tmax)
    it = 1
    for row = 1:nDim:nDim*nAgents;
        for col = 1:1:nDim;
            i = (row-1) + col;
            sumD = u(i,it);
            X(i,it) = sumD*dt;
        end
    end
    T=T+dt;
    it=it+1
end

% Plot
hold on
for row = 1:nDim:nAgents*nDim
    plot3(X(row),X(row+1),X(row+2),t)
end
title('Rendezvous with Helical Leader')
subtitle('PID Dynamics')
xlabel("X_i")
ylabel("X_j")
zlabel("X_z")
hold off
