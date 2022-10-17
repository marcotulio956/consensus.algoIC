% Init
clear % vars
clc   % screen
close all % figures

% Timing
dt=0.01;
Tmax=100;
t0 = -Tmax:dt:Tmax;

X = zeros(2,0);

% Dynamics
A=[0 1; 0 0]; % x1' = x2
B=[0 0; 1 0]; % x2' = u1
C=[1 0]; % output Y mtx
D=zeros(1,2);
sys = ss(A,B,C,D)

% Input
u = ones(2, length(t0));

% Sim
[Y,t]=lsim(sys,u,t0,X);

% Plot
plot(t ,Y, t , u)
title('Double Integrator')
subtitle('1D')
ylabel("y")
xlabel("t")