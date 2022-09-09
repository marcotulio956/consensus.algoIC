clc, clear

X=[2,4,6,8]';
dt=0.001;

Adj=[
    0 1 1 1; 
    1 0 1 1; 
    1 1 0 1; 
    1 1 1 0
];

Din=eye(4);
Lagran=Din-Adj;

Lagran

X

for row = 1:4
    sumDx = 0;
    for col = 1:4
        if col ~= row
            sumDx = sumDx + (Lagran(row,col)*(X(row)-X(col)));
        end
    end
    X(row)=X(row)+sumDx*dt;
end


X