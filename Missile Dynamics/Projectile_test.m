clc; clear; close all;
%%
% Define parameters
global thrust_profile packet1 packet2 gravity_profile
thrust_profile = []; packet1 = []; packet2 = []; gravity_profile = [];
% Initial conditions
initial_state = zeros(15, 1); % replace with your desired initial conditions
initial_state(11) = 30*pi/180;
mach = 343;
initial_state(1) = 2*mach;
initial_state(13) = initial_state(1)*cos(initial_state(11));
initial_state(15) = initial_state(1)*sin(initial_state(11));

% Set time span
tspan = [0, 90]; % replace with your desired time span

% Solve the ODEs using ode45
[t, y] = ode45(@(t, y) flightODE(t, y), tspan, initial_state);

save('Aerodynamic_projectile.mat')
%%

function dydt = flightODE(t, y)
    global thrust_profile packet1 packet2 gravity_profile

    m = 200; % mass
    g = 9.81; % gravitational acceleration
    Radius = 7.62e-2; % Average Radius of Missile (m) 
    Length = 1; % Length of Missile (m)
    I = Missile_Inertia_Matrix(m, Radius, Length);

    % Extract state variables
    u = y(1);
    v = y(2);
    w = y(3);
    x = y(4);
    y_val = y(5);
    z = y(6);
    Q = y(8);
    theta = y(11);
    xdot = y(13); ydot = y(14); zdot = y(15);

    % Calculate thrust vector
    T_Thrust = compute_thrust_projectile(t);
    Thrust = [T_Thrust*cos(theta); 0; T_Thrust*sin(theta)];

    % Calculate gravity vector
    gravity = [0; 0; -m * g]; %* [sin(theta); 0; cos(theta)];
    
    thrust_profile = [thrust_profile T_Thrust];
    gravity_profile = [gravity_profile gravity/m];
    % Equations of motion
    dV = (1/m) * (Thrust + gravity) +...
        [0, 0, Q; 0, 0, 0; -Q, 0, 0] * [xdot; ydot; zdot];

    % Rotation matrix from body frame to inertial frame
    R_bi = [
        cos(theta), 0, -sin(theta);
        0, 1, 0;
        sin(theta), 0, cos(theta)
    ];

    % Rotation matrix from body frame to stability frame
    R_bs = [
        1, 0, -tan(theta);
        0, 1, 0;
        0, 0, 1/cos(theta)
    ];
    inertialVdot = R_bi' * dV;
  
    theta = atan2(zdot,xdot);
    theta_dot = (xdot*inertialVdot(3) -...
        zdot*inertialVdot(1))/xdot^2*cos(theta)^2;

    ang_rates = R_bs'*[0 theta_dot 0]';
    Q = ang_rates(2);

    % Equations for angular rates
    dM = I\(- [0, -0, Q; 0, 0, -0; -Q, 0, 0] * I * [0; Q; 0]);
    packet1 = [packet1 theta];
    % Combine derivatives into a column vector
    dydt = [
        R_bi'*[xdot;ydot;zdot]; % Linear velocities
        [xdot;ydot;zdot]% Linear positions
        dM; % Angular rates
        [0; theta_dot; 0]; % Euler angles
        dV;
    ];
end