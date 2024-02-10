clc; clear; close all;
%%

% Define parameters
global thrust_profile packet1 packet2 gravity_profile states time
thrust_profile = []; packet1 = []; packet2 = []; gravity_profile = []; states = [];
time = [];
% Initial conditions 
initial_state = zeros(15, 1); % replace with your desired initial conditions
V_0 = 2*343;
initial_state(11) = 10*pi/180;
initial_state(1) = V_0;
initial_state(10) = 10*pi/180;

% Set time span
tspan = [0 20]; % replace with your desired time span

% Solve the ODEs using ode45
[t, y] = ode45(@(t, y) flightODE(t, y), tspan, initial_state);

%%
% y = states'; t = time';
save('Aerodynamic_skid.mat')
%%

function dydt = flightODE(t, y)
    global thrust_profile packet1 packet2 gravity_profile states time

    m = 200; % mass
    g = 9.81; % gravitational acceleration
    Radius = 7.62e-2; % Average Radius of Missile (m) 
    Length = 1; % Length of Missile (m)
    I = Missile_Inertia_Matrix(m, Radius, Length);
    thrust_max  = 15e3;

    % Extract state variables
    u = y(1);
    v = y(2);
    w = y(3);
    x = y(4);
    y_val = y(5);
    z = y(6);
    P = y(7);
    Q = y(8);
    R = y(9);
    phi = y(10);
    theta = y(11);
    psi = y(12);
    
    % Aerodynamics
    [temperature, pressure, density, speedOfSound] = troposphereModel(z);
    Mach_number = norm([u, v, w]) / speedOfSound;
    q = 1/8 * density * norm([u, v, w]) * pi * (2 * Radius)^2;
    [Coef_Fxyz, Coef_Momxyz] = Aerodynamic_Model(u, v, w, P, Q, R, Mach_number, theta);
    Aero_forces = q * Coef_Fxyz;
    Aero_moments = -q * Coef_Momxyz * 2*Radius;

    % Calculate thrust vector
    T_Thrust = 0;%compute_thrust(t);
    Thrust = [T_Thrust; 0; 0];

    % Calculate gravity vector
    gravity = -m * g * [sin(theta); sin(phi)*cos(theta); cos(phi)*cos(theta)];
    
    thrust_profile = [thrust_profile Thrust];
    packet1 = [packet1 Aero_forces];
    packet2 = [packet2 Aero_moments];
    gravity_profile = [gravity_profile gravity];
    % Equations of motion
    dV = (1/m) * (Thrust + gravity + Aero_forces) +...
        [0, -R, Q; R, 0, -P; -Q, P, 0] * [u; v; w];

    % Rotation matrix from body frame to inertial frame
    R_bi = [
        cos(theta)*cos(psi), -sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), -cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
        cos(theta)*sin(psi), -sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), -cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
        sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)
    ];

    % Equations for angular rates
    dM = I\(Aero_moments - [0, -R, Q; R, 0, -P; -Q, P, 0] * I * [P; Q; R]);

    % Rotation matrix from body frame to stability frame
    R_bs = [
        1, -sin(phi)*tan(theta), -cos(phi)*tan(theta);
        0, cos(phi), -sin(phi);
        0, sin(phi)/cos(theta), cos(phi)/cos(theta)
    ];
    % Combine derivatives into a column vector
    dydt = [
        dV; % Linear velocities
        R_bi * [u; v; w]; % Linear positions
        dM; % Angular rates
        R_bs * [P; Q; R]; % Euler angles
        R_bi * dV;
    ];
    states = [states y];
    time = [time t];
end

