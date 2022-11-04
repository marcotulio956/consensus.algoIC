Laplacian2=kron(Laplacian, [0 -1; 1 0])% x and x' in X for all agents
%Laplacian2(1:2:end,2:2:end)=eye(nAgents)
Laplacian2(1:nOrder,:)=0 % no dyns 4 leader
Laplacian2 = kron(Laplacian2, eye(nDim)); % generalization to Rn