% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt=0.1;
Tmax=100;
t0 = 0:dt:Tmax;

% Agents Random Initial Positions
nAgents = 3;
nDim = 3;
nOrder = 2;
xAxisL = 10;
yAxisL = 10;
zAxisL = 10;
X = kron([yAxisL.*rand(1,nDim*nAgents)]',[1 0]')
% X(1:nDim*nOrder)=0 % First agent init

% Define Dynamics
Adj = ones(nAgents,nAgents)-eye(nAgents)
leaderAdjWeight=3
Adj(:,1)=leaderAdjWeight*ones(nAgents,1)
Din = -sum(Adj,2).*eye(nAgents) 
Laplacian=Din+Adj
Laplacian(:,1)=leaderAdjWeight*ones(nAgents,1)
Laplacian(1,:)=1*zeros(nAgents,1) % leader
Laplacian=kron(Laplacian, [0 1]) % x x'



Laplacian=repelem(Laplacian,nOrder,1)% temp, 2b ow

accAdjWeight=1;

Laplacian(4:nOrder:end,:)=kron(accAdjWeight.*ones(nAgents-1,nAgents), [0 1]) % acceleration weigths
Laplacian(4:nOrder:end,2)=leaderAdjWeight*ones(nAgents-1,1)

Laplacian=Laplacian-sum(Laplacian,2).*eye(nAgents*nOrder)

Laplacian=reshape(kron(ones(nDim,1), reshape(Laplacian, 2*size(Laplacian,1), [])), size(Laplacian,1), [])

temp = [];
for row = 1:nOrder:nOrder*nAgents
    %temp=[temp;repmat(Laplacian(row,:),3,1)]
    temp = [temp; repmat([Laplacian(row,:); Laplacian(row+1,:)],nDim,1)];
end
Laplacian=temp

% State Space for Agents Positions
A=Laplacian;
B=kron(eye(nAgents*nDim), diag([0 1]))% when u drives leader(only)
C=eye(nOrder*nAgents*nDim)%diag(repmat([1 0],1,nAgents*nDim));% output Y mtx
D=zeros(nAgents*nDim*nOrder);
sys = ss(A,B,C,D)

% Control Input
u=zeros(length(t0),nAgents*nDim*nOrder);
for i = 1:length(t0);
    % straigth line 
    %u(i,1)=-xAxisL*t0(i)/500; 
    %u(i,2)=-yAxisL*t0(i)/500; 
    %u(i,3)=-zAxisL*t0(i)/500;
    % circle
    %u(i,1)=xAxisL*cos(t0(i)*10); 
    %u(i,2)=yAxisL*sin(t0(i)*10); 
    %u(i,3)=zAxisL*sin(t0(i)*10);
    % helice
    u(i,2)=xAxisL/2*cos(t0(i))-yAxisL/2*sin(t0(i));
    u(i,4)=xAxisL/2*sin(t0(i))+yAxisL/2*cos(t0(i));
    u(i,6)=0.8*zAxisL;
end
% Sim
[Y,t]=lsim(sys,u,t0,X);
Y
% Plot
subplot(1,2,1)
title('Second Order Dynamics')
it = [1:height(Y)];
hold on
for agnt = [1:nOrder*nDim:nDim*nAgents-1]
    plot3(Y(it, agnt),Y(it, agnt+2), Y(it, agnt+4))
end
legend('Leader')
xlabel("X_i")
ylabel("X_j")
zlabel("X_k")
hold off

return

subplot(1,2,2)
title('~OTHER~ Dynamics')
it = [1:height(Y2)];
hold on
for agnt2 = [1:nDim:nDim*nAgents]
    plot3(Y2(it, agnt2),Y2(it, agnt2+1), Y2(it, agnt+2))
end
legend('Leader')
xlabel("X_i")
ylabel("X_j")
zlabel("X_k")
hold off

sgtitle('Rendezvous with Helical Leader')