% Init
clear % vars
clc   % screen

% Timing
dt=0.01;
Tmax=3;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions
nAgents = 10;
xAxisL = 5;
X = [xAxisL.*rand(1,nAgents)]'
% X = [2 4 6 8]' % Or specified

% Init Laplacian, must 
Laplacian=[
    0 0 0 0
    1 -3 1 1
    1 1 -3 1
    1 1 1 -3
];
% Or Define Dynamics
Adj = ones(nAgents,nAgents)-eye(nAgents)
Adj(:,1)=nAgents*ones(nAgents,1)
Din = -sum(Adj,2).*eye(nAgents) 

Laplacian=Din+Adj;
Laplacian(:,1)=nAgents*ones(nAgents,1)
Laplacian(1,:)=zeros(1,nAgents)

% State Space for Agents Positions
A=Laplacian;
B=zeros(nAgents,nAgents);
B(1,:)=[1 zeros(1,nAgents-1)];
C=eye(nAgents,nAgents);
D=zeros(nAgents,nAgents);
sys = ss(A,B,C,D)  
% Constant Input for Every Agent(turn on?)
u=1*t0;
u=repmat(u,nAgents,1);
u(1,:)=8*cos(t0*10);

%Sim
[y,t]=lsim(sys,u,t0,X);

%Plotting
plot(t,y)
title("Rendevouz with Random Positions and Sinusoidal Leader (n=10)")
subtitle("1D")
ylabel("X_i")
xlabel("t")
yline(y(1),'-','start');
legend('Leader');
ylim([0 xAxisL])
grid