clear; close all; clc;

% X = [w x y z]
X0 = [1, 2, 7, 9];
tspan = [0 0.2]; % [sec]

[tout1, yout1] = ode45(@(t,X) odefun1(t,X), tspan, X0);
[tout2, yout2] = ode45(@(t,X) odefun2(t,X), tspan, X0);

figure(1);
t1 = tiledlayout(2,2);
title(t1, "1.a");
nexttile
plot(tout1, yout1(:,1))
xlabel("Time")
ylabel("w")
nexttile
plot(tout1, yout1(:,2))
xlabel("Time")
ylabel("x")
nexttile
plot(tout1, yout1(:,3))
xlabel("Time")
ylabel("y")
nexttile
plot(tout1, yout1(:,4))
xlabel("Time")
ylabel("z")

figure(2);
t2 = tiledlayout(2,2);
title(t2, "1.c");
nexttile
plot(tout2, yout2(:,1))
xlabel("Time")
ylabel("w")
nexttile
plot(tout2, yout2(:,2))
xlabel("Time")
ylabel("x")
nexttile
plot(tout2, yout2(:,3))
xlabel("Time")
ylabel("y")
nexttile
plot(tout2, yout2(:,4))
xlabel("Time")
ylabel("z")

function vectorEx = odefun1(t,X)
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

function vectorEx = odefun2(t,X)
    w = X(1);
    x = X(2);
    y = X(3);
    z = X(4);

    w_dot = 9*w + y;
    x_dot = 4*w*x*y - (x^2);
    y_dot = 2*w - x - 2*z;
    z_dot = x*y -(y^2) - 3*(z^3);

    vectorEx = [w_dot x_dot y_dot z_dot]';
end
