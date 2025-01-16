clear; close all; clc;

% constants
m = 50; % [g]
d = 0.02; % [m]
C_D = 0.6; 
h = 1655; % [m]
rho = 1.05; % [kg/m^3] based on h from google :D

density = stdatmo(h);

% 6 dimension initial state vector (3 intial positions & 3 initial velocities)
X0 = [0, % N_0 (initial position in north direction)
      0, % E_0 (initial position in east direction)
      0, % D_0 (initial position in down direction)
      0, % vN_0 (initial velocity in north direction)
      0, % vE_0 (initial velocity in east direction)
      0 % vD_0 (initial velocity in down direction)
       ];

wind = [0, % North direction
        0, % East direction
        0 % Down direction
        ];

% X will be vN, vE, vD, aN, aE, aD

tspan = [0 20]; % [sec]

[tout, yout] = ode45(@(t,X) odefun(t,X), tspan, X0);

tiledlayout(2,2);

nexttile
plot(tout, yout(:,1))
nexttile
plot(tout, yout(:,2))
nexttile
plot(tout, yout(:,3))
nexttile
plot(tout, yout(:,4))


function xdot = objectEOM(t,x,rho,Cd,A,m,g,wind)
% function vectorEx = odefun(t,X)
    w = X(1);
    x = X(2);
    y = X(3);
    z = X(4);

    w_dot = -9*w + y;
    x_dot = 4*w*x*y - (x^2);
    y_dot = 2*w - x - 2*z;
    z_dot = x*y -(y^2) - 3*(z^3);

    vectorEx = [w_dot x_dot y_dot z_dot]';
end
