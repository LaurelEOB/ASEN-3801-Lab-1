clear; close all; clc;
% Laurel O'Brien and Aidan Murray
% ASEN 3801
% ExploringODE45ForSphericalObject
% Created: 1/16/25

% constants
m = 0.05; % [kg]
d = 0.02; % [m]
Cd = 0.6; 
h = 1655; % [m]
A = pi * (d/2)^2; % [m^2]
rho = stdatmo(h); % [kg/m^3]
g = -9.81; % [m/s^2]

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

% X will be vN, vE, vD, aN, aE, aD

tspan = linspace(0,20,2000); % [sec]

options = odeset('Events', @landing); % Makes sure the object stops falling at z=0



%% Question 2.b
[tout_2b, yout_2b] = ode45(@(t,X) objectEOM(t,X, rho, Cd, A, m, g, wind_2b), tspan, X0, options);

figure() % Plotting
scatter3(yout_2b(:,1), yout_2b(:,2), -yout_2b(:,3), 50, tout_2b, "filled") 
xlabel('North (X-axis)')
ylabel('East (Y-axis)')
zlabel('Down (Flipped Z-axis)')
title("2.b No Wind")
ylabel(colorbar, "Time [sec]")



%% Question 2.c
numOfLines = 25; % How many trajectories 

for j = 1:numOfLines % Varies wind 
    wind_vel(j) = (j-1)*2.5; % Finds new wind vel
    wind_2c = [wind_vel(j),0,0]; % N,E,D
    [tout_2c, yout_2c] = ode45(@(t,X) objectEOM(t,X, rho, Cd, A, m, g, wind_2c), tspan, X0, options);
    current__line_a(j) = yout_2c(end,1); % Takes landing point
    current__line_b(j) = sqrt((yout_2c(end,2)^2)+(yout_2c(end,1)^2)); % Takes total distance between origin and landing point
end

% Plotting for horizontal effects
figure('Position', [150 150 600 350]); hold on; grid on;
title({"2.c: Effect of North Facing Wind on Horizontal Landing Location",""});
plot(wind_vel,current__line_a,'b','LineWidth',2);
scatter(wind_vel,current__line_a,30,'b','filled');
xlabel('Wind Speed in North Direction [m/s]')
ylabel({'Horizontal Position of Landing Location [m]',''})

% Plotting for total distance effects
figure('Position', [150 150 600 350]); hold on; grid on;
title({"2.c: Effect of North Facing Wind on Total Distance to Landing Location",""});
plot(wind_vel,current__line_b,'b','LineWidth',2);
scatter(wind_vel,current__line_b,30,'b','filled');
xlabel('Wind Speed in North Direction [m/s]')
ylabel({'Total Distance to Landing Location [m]',''})



%% Question 2.d 
figure('Position', [150 150 800 400]); hold on; grid on; grid on; grid minor;
colororder("default")
line_Colors = get(gca, 'ColorOrder');

for i = 1:6 % Varies altitude by changing the height 
    init_altitude(i) = i*3000/6;
    rho = stdatmo(init_altitude(i));

    for j = 1:25 % Varies wind 
        wind_vel(j) = (j-1)*2.5;
        wind_2d = [wind_vel(j),0,0]; % N,E,D
        [tout_2d, yout_2d] = ode45(@(t,X) objectEOM(t,X, rho, Cd, A, m, g, wind_2d), tspan, X0, options);
        current_altitude_line(i,j) = sqrt((yout_2d(end,2)^2)+(yout_2d(end,1)^2)); % Takes total distance between origin and landing point
    end

    plot(wind_vel,current_altitude_line(i,:),'Color',line_Colors(i,:),'LineWidth',1); % Plots total lines
    plot(wind_vel,current_altitude_line(i,:),'.','Color',line_Colors(i,:),'LineWidth',30); % Plots individual points
    xlim([-2 62])
    ylim([68 75])
    xlabel('Wind Speed in North Direction [m/s]')
    ylabel({'Total Distance to Landing [m]',''})

end

% More Plotting
title("2.d: Effect of Initial Altitude on Total Distance to Landing Location")
legend("Initial Altitude of " +init_altitude(1)+"m",'',"Initial Altitude of " +init_altitude(2)+"m",'',"Initial Altitude of " +init_altitude(3)+"m",'',"Initial Altitude of " +init_altitude(4)+"m",'',"Initial Altitude of " +init_altitude(5)+"m",'',"Initial Altitude of " +init_altitude(6)+"m",'','Location','eastoutside');

% 2nd plot for part d
figure('Position', [150 150 600 350]); hold on; grid on; grid on; grid minor;
title({"2.d: Effect of Initial Altitude on Minimum Total Distance to Landing Location",''})
min_landing_locations = min(current_altitude_line'); % finds min of landing distances
plot(init_altitude,min_landing_locations,'b','LineWidth',2);
scatter(init_altitude,min_landing_locations,30,'b','filled');
xlabel('Initial Altitude [m]')
ylabel({'Minimum Landing Location Distance [m]',''})



%% Question 2.e
rho = stdatmo(h); % [kg/m^3]
KE = 20; % Original KE from 2b [J]
numOfLines = 100;
m_array = [];

for j = 1:numOfLines
    m_2e = 0.001 + 0.001 * (j-1);
    m_array = [m_array m_2e];
    Vmag_2e = sqrt((2*KE)/m_2e);

    % new_KE = (1/2)*m_2e*(Vmag_2e)^2  % verify constant KE

    V_2e = sqrt(((Vmag_2e)^2 / 2));
    
    X0_2e = [0, % N_0 (initial position in north direction)
          0, % E_0 (initial position in east direction)
          0, % D_0 (initial position in down direction)
          0, % vN_0 (initial velocity in north direction)
          V_2e, % vE_0 (initial velocity in east direction)
          -V_2e % vD_0 (initial velocity in down direction)
           ];
       
    wind_2e = [0,0,0]; % N,E,D

    [tout_2e, yout_2e] = ode45(@(t,X) objectEOM(t,X, rho, Cd, A, m_2e, g, wind_2e), tspan, X0_2e, options);
    current__line_2e(j) = sqrt((yout_2e(end,2)^2)+(yout_2e(end,1)^2));

    % For Debugging
    if m_2e == 0.05
        disp(m_2e)
        disp(wind_2e)
        disp(X0_2e)
        disp(yout_2e(end,2))

    end

end

% Plotting 
figure('Position', [150 150 600 350]); hold on; grid on;
title({"2.e: Effect of Mass on Total Distance to Landing Location with Constant Initial Kinetic Energy",""});
plot(m_array,current__line_2e,'b','LineWidth',2);
scatter(m_array,current__line_2e,30,'b','filled');
xlabel('Mass of Object [kg]')
ylabel({'Total Distance to Landing Location [m]',''})




%% Function for ODE45 
function xdot = objectEOM(t,X,rho,Cd,A,m,g,wind)
    %
    % Inputs: t = time
    %         X = state vector = [p_N p_E p_D v_N v_E v_D] {position, velocity]
    %         rho = density
    %         Cd = coefficient of drag
    %         A = cross sectional area
    %         m = mass of object
    %         g = gravity
    %         wing = wing vector = [wind_N wind_E wind_D
    %
    % Outputs: derStateVector = [v_N, v_E, v_D, a_N, a_E, a_D]
    %
    % Methodology: function that returns the derivative of the state vector
    
    % Initial positions
    n = X(1);
    e = X(2);
    d = X(3);
    
    % Initial velocities
    v_n = X(4);
    v_e = X(5);
    v_d = X(6);

    % Relative velocities
    v_rel_n = v_n - wind(1);
    v_rel_e = v_e - wind(2);
    v_rel_d = v_d - wind(3);
    v_rel = [v_rel_n, v_rel_e, v_rel_d];
    v_rel_mag = sqrt(v_rel_n^2 + v_rel_e^2 + v_rel_d^2); % magnitude 

    D = 0.5 * rho * v_rel_mag^2 * A * Cd; % Drag

    f_drag = -D * (v_rel/v_rel_mag); % Force of drag

    f_gravity = [0, 0, m * -g]; % Force of gravity

    f_total = f_drag + f_gravity; % Total force

    a = f_total/m; % Acceleration
   

    xdot = [v_n, v_e, v_d, a(1), a(2), a(3)]'; % Derivatives of state vector
end


%% ODE45 Event function
function [position, isterminal, direction] = landing(t,X)
    % Inputs: t = time
    %         X = state vector = [p_N p_E p_D v_N v_E v_D] {position, velocity]
    %
    % Outputs: Info for ODE45 
    %
    % Methodology: function that makes ODE45 stop at Z=0 at the end

    position = X(3);
    isterminal = 1;
    direction = 0;

end
