nAgents = 4;
xAxisL = 10;
X = [xAxisL.*rand(1,nAgents)]'
% X = [2 4 6 8]' % Or specified

% Dynamics
Din = ones(nAgents,nAgents)-eye(nAgents)
Adj = sum(Din,2).*eye(nAgents)

Laplacian=Din-Adj
Laplacian(1,:)=zeros(1,nAgents)