Laplacian2=kron(Laplacian, [Kp 0; 1 0])% x and sum(x,t) in X for all agents
for i=3:2:height(Laplacian2)
    Laplacian2(i,i+1)=Ki
end
Laplacian2 = kron(Laplacian2, eye(nDim)); % generalization to Rn

Laplacian2