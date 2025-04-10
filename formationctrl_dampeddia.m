% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt=0.01;
Tmax=20;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions
nAgents = 6;
nDim = 3;
nOrder = 2;
xAxisL = 10;
yAxisL = 10;
zAxisL = 10;

% Define Adjacency : both dynamics
Adj = ones(nAgents,nAgents)-eye(nAgents);
leaderAdjWeight=1;
Adj(:,1)=leaderAdjWeight*nAgents*ones(nAgents,1);
Din = -sum(Adj,2).*eye(nAgents);
Laplacian=Din+Adj;
% Laplacian(:,1)=1*ones(nAgents,1) % leader adj to all
Laplacian(1,:)=zeros(1,nAgents) % path1 dyn
Laplacian(2,:)=zeros(1,nAgents) % path2 dyn
Laplacian(3,:)=zeros(1,nAgents) % path3 dyn

%Laplacian = [0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;6,0,0,-8,1,1;0,6,0,1,-8,1;0,0,6,1,0,-8];
Laplacian = [0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;6,0,0,-6,0,0;0,6,0,0,-6,0;0,0,6,0,0,-6];

% Define Dynamics : D integrator with damping
X1 = [min([xAxisL yAxisL zAxisL]).*rand(1,nDim*nAgents*nOrder)]'
for i=nDim+1:2*nDim:height(X1)% initial acc as 0
    X1(i:i+nDim-1)=0;
end
X1(1)=4; % path1
X1(2)=0;
X1(3)=0;
X1(4)=0; 
X1(5)=4;
X1(6)=0;
X1(7)=2;% path2
X1(8)=0;
X1(9)=4;
X1(10)=0;
X1(11)=0;
X1(12)=0;
X1(13)=0;% path2
X1(14)=0;
X1(15)=0;
X1(16)=0;
X1(17)=4;
X1(18)=0;


Laplacian1=kron(Laplacian, [0.09 0; 1 0])% x and x' in X for all agents
Laplacian1(1:2:end,2:2:end)=eye(nAgents)
Laplacian1 = kron(Laplacian1, eye(nDim)); % generalization to Rn

% State Space for Agents Positions : integrator
A1=Laplacian1;
B1=eye(nOrder*nAgents*nDim);
C1=zeros(nOrder*nAgents*nDim); % output Y mtx
for i=1:2*nDim:height(C1)
    C1(i:i+nDim-1,i:i+nDim-1)=eye(nDim);
end
D1=zeros(nAgents*nDim*nOrder);
sys1 = ss(A1,B1,C1,D1)

% Control Input
%u=zeros(nAgents*nDim,length(t0));
u1=zeros(length(t0),nDim*nAgents*nOrder);
u2=zeros(length(t0),nDim*nAgents*nOrder);
u3=zeros(length(t0),nDim*nAgents*nOrder);
%u(1,:)=(xAxisL/2)*(1+sin(t0)); % Leader
%u(2,:)=(yAxisL/2)*(1+cos(t0));
for i = 1:length(t0)
    % straigth line 
    %u(i,1)=-xAxisL*t0(i)/500; 
    %u(i,2)=-yAxisL*t0(i)/500; 
    %u(i,3)=-zAxisL*t0(i)/500;
    % circle
    %u(i,1)=xAxisL*cos(t0(i)*10); 
    %u(i,2)=yAxisL*sin(t0(i)*10); 
    %u(i,3)=zAxisL*sin(t0(i)*10);
    % helice
    %u2(i,1)=xAxisL/2*cos(t0(i))-yAxisL/2*sin(t0(i));
    %u2(i,2)=xAxisL/2*sin(t0(i))+yAxisL/2*cos(t0(i));
    %u2(i,3)=0.8*zAxisL*t0(i);
    %%u2(i,1)=ones(length(t0(i)));
    %u1(i,2)=zeros(length(t0(i)));
    %u1(i,3)=zeros(length(t0(i)));
    u2(i,8)=ones(length(t0(i)));
    %u2(i,2)=zeros(length(t0(i)));
    %u2(i,3)=zeros(length(t0(i)));
    %%u2(i,15)=ones(length(t0(i)));
    %u3(i,2)=zeros(length(t0(i)));
    %u3(i,3)=zeros(length(t0(i)));
end

% Sim
[Y,t]=lsim(sys1,u2,t0,X1);

% Plot
%subplot(1,2,1)
subtitle('Damped Double Integrator')
it = 1:height(Y);
hold on
for agnt = 1:nDim*nOrder:nDim*nAgents*nOrder
    %if agnt >= 19
    plot3(Y(it, agnt), Y(it, agnt+1), Y(it, agnt+2), 'LineWidth', 2)
    % Mark the initial position of the agent with a black circle
    plot3(Y(1, agnt), Y(1, agnt+1), Y(1, agnt+2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
  
    %end
end
legend('path1','','path2','','path3')
xlabel("X_i")
ylabel("X_j")
zlabel("X_k")
hold off

title('Triangle Formation')