% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt=0.01;
Tmax=10;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions
nAgents = 1;
nDim = 3;
xAxisL = 10;
yAxisL = 10;
zAxisL = 10;
X = [min(xAxisL,yAxisL).*rand(1,nDim*nAgents)]';
X(1)=xAxisL/2;
X(2)=yAxisL/2;
X(3)=zAxisL*0.7;

A=zeros(3,3);
B=eye(3);
C=eye(3);
D=zeros(3,3);
sys = ss(A,B,C,D)

%u=zeros(1, length(t0)); % length(t0),nAgents*nDim
%u=repmat(u,nDim*nAgents,1);
u=zeros(1);
u=repmat(u,length(t0),3);
for i = 1:length(t0)
    u(i,1)=xAxisL*sin(t0(i)*10); 
    u(i,2)=yAxisL*cos(t0(i)*10); 
    u(i,3)=0;
end

sys, X, u

% Sim
[Y,t]=lsim(sys,u,t0,X);

it = [1:height(Y)];

hold on
for agnt = [1:nDim:nDim*nAgents]
    plot3(Y(it, agnt),Y(it, agnt+1), Y(it, agnt+2))
end

legend('Leader')
title("Rendezvous with Circular Leader")
subtitle("3D")
xlabel("X_i")
ylabel("X_j")
zlabel("X_k")
Y
t0