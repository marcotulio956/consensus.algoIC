% Init
clear % vars
clc   % screen

% Sim
format short
dt = 0.001;
T=0;
Tmax=1;

% Agents Random Initial Positions
nAgents = 10;
xAxisL = 10;
X = [xAxisL.*rand(1,nAgents)]';
% X = [2 4 6 8]'

% State Space for Agents Positions
Adj=[
    0 1 0 1; 
    1 0 1 0; 
    0 0 0 0; 
    1 0 0 0
]; % 1:2,4 ; 2:1,3; 3: ; 4:1
Adj=(eye(nAgents)+1)-eye(nAgents)% full adj

Din=eye(nAgents);
Lagran=Din-Adj;

X, Adj, Lagran

XSeries = timeseries(X,T);
XSeries.Name = '1D Redezvous'

while (T<Tmax)
    for row = 1:nAgents
        sumDx = 0;
        for col = 1:nAgents
            if col ~= row
                sumDx = sumDx + (Lagran(row,col)*(X(row)-X(col)));
            end
        end
        X(row)=X(row)+sumDx*dt;
    end
    
    T=T+dt;
    XSerie  = timeseries(X,T);
    XSeries = append(XSeries,XSerie);
end

plot(XSeries)
title("Rendezvous with Random Positions")
subtitle("1D")
ylabel("Xi")
xlabel("t")