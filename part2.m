clear; close all; clc;

% constants
m = 0.05; % [kg]
d = 0.02; % [m]
Cd = 0.6; 
h = 1655; % [m]
A = pi * (d/2)^2; % [m^2]
rho = stdatmo(h);
g = -9.81;

% 6 dimension initial state vector (3 intial positions & 3 initial velocities)
X0 = [0, % N_0 (initial position in north direction)
      0, % E_0 (initial position in east direction)
      0, % D_0 (initial position in down direction)
      0, % vN_0 (initial velocity in north direction)
      20, % vE_0 (initial velocity in east direction)
      -20 % vD_0 (initial velocity in down direction)
       ];

wind_2b = [0, % North direction
        0, % East direction
        0% Down direction
        ];

wind_2c = [30, % North direction
        0, % East direction
        0% Down direction
        ];

% X will be vN, vE, vD, aN, aE, aD

tspan = linspace(0,20,2000);
% tspan = [0 20]; % [sec]

options = odeset('Events', @landing);

[tout_2b, yout_2b] = ode45(@(t,X) objectEOM(t,X, rho, Cd, A, m, g, wind_2b), tspan, X0, options);



[tout_2c, yout_2c] = ode45(@(t,X) objectEOM(t,X, rho, Cd, A, m, g, wind_2c), tspan, X0, options);



% Question 2.b
figure()

scatter3(yout_2b(:,1), yout_2b(:,2), -yout_2b(:,3), 50, tout_2b, "filled") 

xlabel('North')
ylabel('East')
zlabel('Down (flipped)')
title("2.b No Wind")

ylabel(colorbar, "time")


% Question 2.c.a
figure()
scatter3(yout_2c(:,1), yout_2c(:,2), -yout_2c(:,3), 50, tout_2c, "filled") 

xlabel('North')
ylabel('East')
zlabel('Down (flipped)')
title("2.c 20m/s Wind to the North")

ylabel(colorbar, "time")

function xdot = objectEOM(t,X,rho,Cd,A,m,g,wind)
% function vectorEx = odefun(t,X)
    
    n = X(1);
    e = X(2);
    d = X(3);
    
    v_n = X(4);
    v_e = X(5);
    v_d = X(6);

    v_rel_n = v_n - wind(1);
    v_rel_e = v_e - wind(2);
    v_rel_d = v_d - wind(3);
    
    v_rel = [v_rel_n,
           v_rel_e,
           v_rel_d];

    v_rel_mag = sqrt(v_rel_n^2 + v_rel_e^2 + v_rel_d^2);

    D = 0.5 * rho * v_rel_mag^2 * A * Cd;

    f_drag = -D * (v_rel/v_rel_mag);

    f_gravity = [0,
                 0,
                 m * -g];

    f_total = f_drag + f_gravity;


    a = f_total/m;
   

    xdot = [v_n, v_e, v_d, a(1), a(2), a(3)]';
end


function [position, isterminal, direction] = landing(t,X)

    position = X(3);
    isterminal = 1;
    direction = 0;

end