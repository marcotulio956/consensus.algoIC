% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt=0.01;
Tmax=1;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions
nAgents = 1000;
nDim = 2;
xAxisL = 10;
yAxisL = 10;
X = [min(xAxisL,yAxisL).*rand(1,2*nAgents)]';

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
B=zeros(nAgents*nDim,nAgents*nDim);
B(1,:)=[ones(1,nDim) zeros(1,nAgents*nDim-2)]; % when u is driven for leader(only)
C=eye(nAgents*nDim,nAgents*nDim); % output Y mtx
D=zeros(nAgents*nDim,nAgents*nDim);
sys = ss(A,B,C,D)

% Control Input
u=zeros(nAgents*nDim,length(t0));
%u(1,:)=(xAxisL/2)*(1+sin(t0)); % Leader
%u(2,:)=(yAxisL/2)*(1+cos(t0));
%u(1,:)=sin((1/100)*t0)
%u(2,:)=cos((1/100)*t0)
%u

% Sim
[Y,t]=lsim(sys,u,t0,X);

it = [1:height(Y)];

hold on
plot3(Y(it, 1),Y(it, 2), it, ':')
for agnt = [3:2:2*nAgents]
    plot3(Y(it, agnt),Y(it, agnt+1), it)
end
legend('Leader')
title("Rendezvous with Leader")
subtitle("2D")
xlabel("X_i")
ylabel("X_j")
zlabel("t")

