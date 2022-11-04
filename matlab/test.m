% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt=0.01;
Tmax=10;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions
nAgents = 4;
nDim = 3;
xAxisL = 10;
yAxisL = 10;
zAxisL = 10;
X1 = [min([xAxisL yAxisL zAxisL]).*rand(1,nDim*nAgents)]';
X1(1)=0;
X1(2)=0;
X1(3)=0;

% Define Adjacency : both dynamics
Adj = ones(nAgents,nAgents)-eye(nAgents);
leaderAdjWeight=1;
Adj(:,1)=leaderAdjWeight*nAgents*ones(nAgents,1);
Din = -sum(Adj,2).*eye(nAgents);
Laplacian=Din+Adj;
% Laplacian(:,1)=1*ones(nAgents,1) % leader adj to all
Laplacian(1,:)=zeros(1,nAgents) % leader dyn

% Define Dynamics : D integrator
nOrder = 2;

X2 = [min([xAxisL yAxisL zAxisL]).*rand(1,nDim*nAgents*nOrder)]'
for i=nDim+1:2*nDim:height(X2)% initial acc as 0
    X2(i:i+nDim-1)=0;
end 
X2(1)=0; % leader
X2(2)=0;
X2(3)=0;

Laplacian2=kron(Laplacian, [0 0; 1 0])% x and x' in X for all agents

Laplacian2(1:2:end,2:2:end)=-1.*eye(nAgents)
Laplacian2(1:nOrder,:)=0 % no dyns 4 leader
Laplacian2 = kron(Laplacian2, eye(nDim)); % generalization to Rn

% State Space for Agents Positions : D integrator
A2=Laplacian2;
%B=zeros(nAgents*nDim,nAgents*nDim);
B2=eye(nOrder*nAgents*nDim);
C2=zeros(nOrder*nAgents*nDim); % output Y mtx
for i=1:2*nDim:height(C2)
    C2(i:i+nDim-1,i:i+nDim-1)=eye(nDim);
end
D2=zeros(nAgents*nDim*nOrder);
sys2 = ss(A2,B2,C2,D2)