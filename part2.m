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

% 6 dimension initial state vector (3 intial positions & 3 initial velocities)
X0_2d = [0, % N_0 (initial position in north direction)
      0, % E_0 (initial position in east direction)
      0, % D_0 (initial position in down direction)
      0, % vN_0 (initial velocity in north direction)
      50, % vE_0 (initial velocity in east direction)
      -50 % vD_0 (initial velocity in down direction)
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

xlabel('North (X-axis)')
ylabel('East (Y-axis)')
zlabel('Down (Flipped Z-axis)')
title("2.b No Wind")

ylabel(colorbar, "Time [sec]")


% Question 2.c.a
figure()
scatter3(yout_2c(:,1), yout_2c(:,2), -yout_2c(:,3), 50, tout_2c, "filled") 

xlabel('North (X-axis)')
ylabel('East (Y-axis)')
zlabel('Down (Flipped Z-axis)')
title("2.c 20m/s Wind to the North")

ylabel(colorbar, "Time [sec]")


% Question 2.d ** Looked interesting, but not correct 
% figure('Position', [150 150 900 300]); hold on; view(90,0);
% for i = 1:4
%     rho = stdatmo(i*3000/4);
%     lineColors = [[0.4660 0.6740 0.1880];[0.3010 0.7450 0.9330];[0 0.4470 0.7410];[0.4940 0.1840 0.5560]];
%     [tout_2d, yout_2d] = ode45(@(t,X) objectEOM(t,X, rho, Cd, A, m, g, wind_2b), tspan, X0_2d, options);
%     plot3(yout_2d(:,1), yout_2d(:,2), -yout_2d(:,3),"LineWidth",2,"Color",lineColors(i,:))
% end
% xlabel('North (X-axis)')
% ylabel('East (Y-axis)')
% zlabel('Down (Flipped Z-axis)')
% title("2.d Impact of Initial Altitude on Landing Location")
% legend("750m Altitude","1500m Altitude","2250m Altitude","3000m Altitude");
% annotation('textbox', [.777 .4 .145 .25], 'string', {"V_0_,_E= 50m/s","V_0_,_D= -50m/s","No Wind"})

% Question 2.d 
figure('Position', [150 150 600 500]); 
t1 = tiledlayout(4,1);
for i = 1:4
    nexttile; hold on; grid on; grid on; grid minor;
    init_altitude = i*3000/4;
    rho = stdatmo(init_altitude);
    for j = 1:7
        wind_vel = (j-1)*10;
        wind_2d = [wind_vel,wind_vel,-wind_vel]; % N,E,D
        lineColors = [[0.9290 0.6940 0.1250];[0.4660 0.6740 0.1880];[0.3010 0.7450 0.9330];[0 0.4470 0.7410];[0.4940 0.1840 0.5560]];
        [tout_2d, yout_2d] = ode45(@(t,X) objectEOM(t,X, rho, Cd, A, m, g, wind_2d), tspan, X0_2d, options);
        scatter(yout_2d(end,1),yout_2d(end,2),'filled',"LineWidth",2)
        
    end
    scatter(0,0,'+k')
    xlim([-30 800])
    ylim([0 1200])
    xlabel('North (X-axis)')
    ylabel('East (Y-axis)')
    title("Initial Altitude of " +init_altitude+"m")
end



legend(" 0m/s Wind Landing Location","10m/s Wind Landing Location","20m/s Wind Landing Location","30m/s Wind Landing Location","40m/s Wind Landing Location","50m/s Wind Landing Location","60m/s Wind Landing Location","Origin",'Location','eastoutside');
%annotation('textbox', [.777 .4 .145 .25], 'string', {"V_0_,_E= 50m/s","V_0_,_D= -50m/s","No Wind"})
zlim([-1 1])




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
