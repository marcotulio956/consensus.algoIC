% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt=0.01;
Tmax=10;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions
nAgents = 7;
nDim = 2;
xAxisL = 10;
yAxisL = 10;
X = [min(xAxisL,yAxisL).*rand(1,2*nAgents)]'

% Define Dynamics
Adj = ones(nAgents,nAgents)-eye(nAgents)
leaderAdjWeight=1
Adj(:,1)=leaderAdjWeight*nAgents*ones(nAgents,1)
Din = -sum(Adj,2).*eye(nAgents) 

Laplacian=Din+Adj;
Laplacian(:,1)=1*ones(nAgents,1)
Laplacian(1,:)=zeros(1,nAgents)

Laplacian = kron(Laplacian, eye(nDim))

% State Space for Agents Positions
A=Laplacian;
B=zeros(nAgents*nDim,nAgents*nDim);
%B(1,:)=[1 1 zeros(1,nAgents*nDim-2)]; % when u is driven for leader
C=eye(nAgents*nDim,nAgents*nDim);
D=zeros(nAgents*nDim,nAgents*nDim);
sys = ss(A,B,C,D)  

% Constant Input for Every Agent(turn on?)
u=1*t0;
u=repmat(u,nAgents*nDim,1);

%Sim
[Y,t]=lsim(sys,u,t0,X);

%2D Plot
% Y
it = [1:height(Y)];
Y
for agnt = [1:nAgents-1]
    for it = [1:height(Y)]
        plot3(Y(it, agnt),Y(it, agnt+1), it, '.')
        hold on
    end
end
