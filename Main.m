clear; close all; clc;

% X = [w x y z]
X0 = [5, 2, 7, 9];
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

function vectorEx = odefun(t,X)
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
