% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt=0.01;
Tmax=100;
t0 = 0:dt:Tmax;

X = zeros(2,0);

% Dynamics
c=1.1, m=1, k=10; % viscuous friction coeff, mass, spring constant
A=[0 1; -k/m -c/m];
B=[0 1/m; 0 0];
C=[9 0]; % output Y mtx
D=zeros(1,2);
sys = ss(A,B,C,D)

% Input
u = ones(2, length(t0));

% Sim
[Y,t]=lsim(sys,u,t0,X);

% Plot
plot(t ,Y, t , u)
title('Harmonic Oscillator')
subtitle('1D')
ylabel("y")
xlabel("t")