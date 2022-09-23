% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt=0.01;
Tmax=15;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions
nAgents = 50;
nDim = 3;
xAxisL = 10;
yAxisL = 10;
zAxisL = 10;
X = [min([xAxisL yAxisL zAxisL]).*rand(1,nDim*nAgents)]';
X(1)=xAxisL/2;
X(2)=yAxisL/2;
X(3)=zAxisL*0.9;

% Define Dynamics
Adj = ones(nAgents,nAgents)-eye(nAgents)
leaderAdjWeight=1
Adj(:,1)=leaderAdjWeight*nAgents*ones(nAgents,1)
Din = -sum(Adj,2).*eye(nAgents) 
Laplacian=Din+Adj;
% Laplacian(:,1)=1*ones(nAgents,1) % leader adj to all
Laplacian(1,:)=zeros(1,nAgents) % leader dyn
Laplacian = kron(Laplacian, eye(nDim)) % generalization to Rn

% State Space for Agents Positions
A=Laplacian;
%B=zeros(nAgents*nDim,nAgents*nDim);
%B(1,:)=[ones(1,nDim) zeros(1,nAgents*nDim-2)]; % when u is driven for leader(only)
B=eye(nAgents*nDim)
C=eye(nAgents*nDim); % output Y mtx
D=zeros(nAgents*nDim);
sys = ss(A,B,C,D)

% Control Input
%u=zeros(nAgents*nDim,length(t0));
u=zeros(1);
u=repmat(u,length(t0),nDim*nAgents);
%u(1,:)=(xAxisL/2)*(1+sin(t0)); % Leader
%u(2,:)=(yAxisL/2)*(1+cos(t0));
for i = 1:length(t0)
    u(i,1)=xAxisL*cos(t0(i)*10); 
    u(i,2)=yAxisL*sin(t0(i)*10); 
    u(i,3)=zAxisL*sin(t0(i)*10);
end

% Sim
[Y,t]=lsim(sys,u,t0,X);

it = [1:height(Y)];

hold on
for agnt = [1:nDim:nDim*nAgents-1]
    plot3(Y(it, agnt),Y(it, agnt+1), Y(it, agnt+2))
end

legend('Leader')
title("Rendezvous with Circular Leader")
subtitle("3D")
xlabel("X_i")
ylabel("X_j")
zlabel("X_k")

