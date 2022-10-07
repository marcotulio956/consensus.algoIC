% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt=0.01;
Tmax=5;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions
nAgents = 3;
nDim = 3;
nOrder = 2;
xAxisL = 10;
yAxisL = 10;
zAxisL = 10;
X = [min([xAxisL yAxisL zAxisL]).*rand(1,nDim*nAgents*nOrder)]';
X(1:nDim*nOrder)=0;

% Define Dynamics
Adj = ones(nAgents,nAgents)-eye(nAgents)
leaderAdjWeight=1;
Adj(:,1)=leaderAdjWeight*ones(nAgents,1)
Din = -sum(Adj,2).*eye(nAgents) 
Laplacian=Din+Adj;
% Laplacian(:,1)=1*ones(nAgents,1) % leader adj to all
Laplacian(1,:)=zeros(1,nAgents); % leader dyn
Laplacian*agntDym
Laplacian = kron(Laplacian, [0 1]) % order 


Laplacian = kron(Laplacian, eye(nDim)) % generalization to Rn


% State Space for Agents Positions
A=Laplacian;
%B=zeros(nAgents*nDim,nAgents*nDim);
%B(1,:)=[ones(1,nDim) zeros(1,nAgents*nDim-2)]; % when u is driven for leader(only)
B=eye(nAgents*nDim);
C=eye(nAgents*nDim); % output Y mtx
D=zeros(nAgents*nDim);
sys = ss(A,B,C,D)