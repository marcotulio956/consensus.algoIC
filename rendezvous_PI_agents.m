% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt=0.01;
Tmax=5;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions
nAgents = 4;
nDim = 3;
xAxisL = 10;
yAxisL = 10;
zAxisL = 10;
X1 = [min([xAxisL yAxisL zAxisL]).*rand(1,nDim*nAgents)]';
X(1)=0;
X(2)=0;
X(3)=0;

% Define Adjacency : both dynamics
Adj = ones(nAgents,nAgents)-eye(nAgents);
leaderAdjWeight=1;
Adj(:,1)=leaderAdjWeight*nAgents*ones(nAgents,1);
Din = -sum(Adj,2).*eye(nAgents);
Laplacian=Din+Adj;
% Laplacian(:,1)=1*ones(nAgents,1) % leader adj to all
Laplacian(1,:)=zeros(1,nAgents) % leader dyn

% P --------------
% State Space for Agents Positions
Laplacian1 = kron(Laplacian, eye(nDim)); % generalization to Rn
A1=Laplacian1;
%B=zeros(nAgents*nDim,nAgents*nDim);
%B(1,:)=[ones(1,nDim) zeros(1,nAgents*nDim-2)]; % when u is driven for leader(only)
B1=eye(nAgents*nDim);
C1=eye(nAgents*nDim); % output Y mtx
D1=zeros(nAgents*nDim);
sys1 = ss(A1,B1,C1,D1);

% PI --------------
nCtrl = 2;
Ki=30; Kp=10;
X2 = [min([xAxisL yAxisL zAxisL]).*rand(1,nDim*nAgents*nCtrl)]';
% State Space for Agents Positions
Laplacian2=kron(Laplacian, [Kp 0; 1 0])% x and sum(x,t) in X for all agents
for i=3:2:height(Laplacian2)
    Laplacian2(i,i+1)=Ki
end
Laplacian2 = kron(Laplacian2, eye(nDim)); % generalization to Rn
A2=Laplacian2;
%B=zeros(nAgents*nDim,nAgents*nDim);
%B(1,:)=[ones(1,nDim) zeros(1,nAgents*nDim-2)]; % when u is driven for leader(only)
B2=eye(nAgents*nDim*nCtrl)
C2=eye(nAgents*nDim*nCtrl); % output Y mtx
D2=zeros(nAgents*nDim*nCtrl);
sys2 = ss(A2,B2,C2,D2);

% Control Input
%u=zeros(nAgents*nDim,length(t0));
u1=zeros(1);
u1=repmat(u1,length(t0),nDim*nAgents);
%u(1,:)=(xAxisL/2)*(1+sin(t0)); % Leader
%u(2,:)=(yAxisL/2)*(1+cos(t0));
for i = 1:length(t0)
    % straight line 
    %u(i,1)=-xAxisL*t0(i)/500; 
    %u(i,2)=-yAxisL*t0(i)/500; 
    %u(i,3)=-zAxisL*t0(i)/500;
    % circle
    %u(i,1)=xAxisL*cos(t0(i)*10); 
    %u(i,2)=yAxisL*sin(t0(i)*10); 
    %u(i,3)=zAxisL*sin(t0(i)*10);
    % helice
    u1(i,1)=xAxisL/2*cos(t0(i))-yAxisL/2*sin(t0(i));
    u1(i,2)=xAxisL/2*sin(t0(i))+yAxisL/2*cos(t0(i));
    u1(i,3)=0.8*zAxisL;
end

%u=zeros(nAgents*nDim,length(t0));
u2=zeros(1);
u2=repmat(u2,length(t0),nDim*nAgents*nCtrl);
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
    u2(i,1)=xAxisL/2*cos(t0(i))-yAxisL/2*sin(t0(i));
    u2(i,2)=xAxisL/2*sin(t0(i))+yAxisL/2*cos(t0(i));
    u2(i,3)=0.8*zAxisL;
end

% Sim
[Y,t]=lsim(sys1,u1,t0,X1); % P
[Y2,t2]=lsim(sys2,u2,t0,X2); % PI

% Plot
subplot(1,2,1)
title('P Control')
it = [1:height(Y)];
hold on
for agnt = [1:nDim:nDim*nAgents-1]
    plot3(Y(it, agnt),Y(it, agnt+1), Y(it, agnt+2))
end
legend('Leader')
xlabel("X_i")
ylabel("X_j")
zlabel("X_k")
hold off

subplot(1,2,2)
title('PI Control')
it = [1:height(Y2)];
hold on
for agnt2 = 1:nDim*nCtrl:nDim*nAgents*nCtrl
    plot3(Y2(it, agnt2),Y2(it, agnt2+1), Y2(it, agnt2+2))
end
legend('Leader')
xlabel("X_i")
ylabel("X_j")
zlabel("X_k")
hold off

sgtitle('Rendezvous with Random Positions and Helical Leader (n=4)')
subtitle('3D')