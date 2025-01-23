clear; close all; clc;
% Laurel O'Brien and Aidan Murray
% ASEN 3801
% ExploringODE45ForDynamicSysEq
% Created: 1/16/25

% X = [w x y z]
X0 = [1, 2, 7, 9];
tspan1 = [0 20]; % [sec]
tspan2 = [0 0.2]; % [sec]
tspan3 = [0 2]; % [sec]

[tout1, yout1] = ode45(@(t,X) odefun1(t,X), tspan1, X0);
[tout2, yout2] = ode45(@(t,X) odefun2(t,X), tspan2, X0);
[tout3, yout3] = ode45(@(t,X) odefun2(t,X), tspan3, X0);

font_size = 13;
line_width = 2;

figure('Position', [50 50 350 600]);
t1 = tiledlayout(4,1);
title(t1, '$1.a:  \dot{w} = -9w+y$', 'Interpreter', 'latex')
nexttile; plot(tout1, yout1(:,1),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("w","FontSize",font_size+2,"Rotation",0);
nexttile; plot(tout1, yout1(:,2),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("x","FontSize",font_size+2,"Rotation",0);
nexttile; plot(tout1, yout1(:,3),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("y","FontSize",font_size+2,"Rotation",0);
nexttile; plot(tout1, yout1(:,4),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("z","FontSize",font_size+2,"Rotation",0);

figure('Position', [450 50 350 600]);
t2 = tiledlayout(4,1);
title(t2, '$1.c:  \dot{w} = 9w+y$', 'Interpreter', 'latex');
nexttile; plot(tout2, yout2(:,1),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("w","FontSize",font_size+2,"Rotation",0);
nexttile; plot(tout2, yout2(:,2),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("x","FontSize",font_size+2,"Rotation",0);
nexttile; plot(tout2, yout2(:,3),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("y","FontSize",font_size+2,"Rotation",0);
nexttile; plot(tout2, yout2(:,4),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("z","FontSize",font_size+2,"Rotation",0);

figure('Position', [750 50 350 600]);
t3 = tiledlayout(4,1);
title(t3, '$1.d:  \dot{w} = 9w+y$', 'Interpreter', 'latex');
nexttile; plot(tout3, yout3(:,1),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("w","FontSize",font_size+2,"Rotation",0);
nexttile; plot(tout3, yout3(:,2),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("x","FontSize",font_size+2,"Rotation",0);
nexttile; plot(tout3, yout3(:,3),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("y","FontSize",font_size+2,"Rotation",0);
nexttile; plot(tout3, yout3(:,4),LineWidth=line_width);
xlabel("Time [sec]","FontSize",font_size); ylabel("z","FontSize",font_size+2,"Rotation",0);

function derStateVector = odefun1(~,X)
%
% Inputs: t = time
%         X = state vector = [w x y z]
%
% Outputs: derStateVector = [w_dot x_dot y_dot z_dot]
%
% Methodology: function that returns the derivative of the state vector
    w = X(1);
    x = X(2);
    y = X(3);
    z = X(4);

    w_dot = -9*w + y; % w_dot for part 1a
    x_dot = 4*w*x*y - (x^2);
    y_dot = 2*w - x - 2*z;
    z_dot = x*y -(y^2) - 3*(z^3);

    derStateVector = [w_dot x_dot y_dot z_dot]';
end

function derStateVector = odefun2(~,X)
%
% Inputs: t = time
%         X = state vector = [w x y z]
%
% Outputs: derStateVector = [w_dot x_dot y_dot z_dot]
%
% Methodology: function that returns the derivative of the state vector
    w = X(1);
    x = X(2);
    y = X(3);
    z = X(4);

    w_dot = 9*w + y; % w_dot for part 1c
    x_dot = 4*w*x*y - (x^2);
    y_dot = 2*w - x - 2*z;
    z_dot = x*y -(y^2) - 3*(z^3);

    derStateVector = [w_dot x_dot y_dot z_dot]';
end
