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
b=1, j=1; % viscuous friction coeff, inertia
A=[0 1; 0 -b/j];
B=[0 1/j; 0 0];
C=[1 1]; % output Y mtx
D=zeros(1,2);
sys = ss(A,B,C,D)

% Input
u = [0:dt:Tmax;0:dt:Tmax] % Ramp

% Sim
[Y,t]=lsim(sys,u,t0,X);

% Plot
plot(t ,Y, t , u) % Parabola
title('DC Motor')
subtitle('1D')
ylabel("\omega")
xlabel("t")