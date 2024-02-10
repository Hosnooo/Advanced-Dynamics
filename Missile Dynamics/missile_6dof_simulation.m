clear; close all; clc;
%% Initialization
global thrust_profile gravity_profile time count
thrust_profile = []; gravity_profile = []; time = []; count = 0;

% Initial conditions
one_Mach = 343;
x0 = 0; y0 = 0; z0 = 0;  % Initial position (m)
u0 = 0; v0 = 0; w0 = 0;  % Initial velocity (m/s)
phi0 = 10*pi/180; theta0 = 0; psi0 = 0;  % Initial orientation (radians)
P0 = 0; Q0 = 0; R0 = 0;  % Initial angular rates (rad/s)

initial_state = [x0, y0, z0, u0, v0, w0, phi0, theta0, psi0, P0, Q0, R0];

% Time settings
t_span = [0, 4.5*60];  % Simulation time span (s)

% Trackers
trackers = struct('time', [], 'thrust', [], 'aero_forces', [], 'aero_moments', [], ...
    'accelerations', [], 'speeds', [], 'positions', []);

%% Solve the equations of motion
% [t, state] = ode45(@(t, y) missile_6dof_dynamics(t, y), t_span, initial_state);
[t, state] = ode45(@(t, y) missile_dynamics_projectile(t, y), t_span, [initial_state(1:7)...
    0 0 0]);

%% Plot the results
plot_results(t, state);

% Creating a new vector without elements in the specified range
% thrust_profile = [thrust_profile(1:50-1), thrust_profile(57+1:end)];
% thrust_profile = thrust_profile(1:length(t));
% figure()
% plot(t,...
%     thrust_profile/1000,'-.','LineWidth', 1.2)
% grid on;
% xlabel('time (sec)')
% ylabel('Force (Kilo-Newton)')
% title('Thrust Profile')
% limitIncreaseFactor = 0.2;
% newLimits = edit_limits(axis, limitIncreaseFactor);
%     axis(newLimits);
%     figure()
% plot(gravity_profile')
%% Subfunctions
%% Projectile Motion Assumbtion

function dydt = missile_dynamics_projectile(t, y)
    % Projectile dynamics model (basic example)
    global thrust_profile gravity_profile time count
    count = count + 1;
    % Placeholder values for projectile parameters (replace with your specific values)
    projectile_mass = 200;  % Mass of the projectile (kg)
    gravity = 9.81;        % Acceleration due to gravity (m/s^2)
    
    % Extract states
    x_pos = y(1); y_pos = y(2); z_pos = y(3);
    u = y(4); v = y(5); w = y(6);
    theta = y(7);

    % Projectile dynamics equations
    dydt = zeros(10, 1);

    % Translational motion
    dydt(1:3) = Rotate(-theta,'y')'*[u; v; w];

    % Gravitational force
    R_gravity = Rotate(-theta,'y');
    gravity_force = R_gravity*[0; 0; - projectile_mass * gravity];
    
    % Acceleration
    Thrust = compute_thrust(t);
    dydt(4:6) = (gravity_force + [Thrust; 0; 0]) / projectile_mass;
    
    dydt(8:10) = Rotate(-theta,'y')'*dydt(4:6);
    % dydt(8:10)
    dydt(11) = I \ (- Angular_Matrix * I * [P; Q; R]);
    % Theta
    xdot = dydt(1); zdot = dydt(3); xddot = dydt(8); zddot = dydt(10);
    dydt(7) = (x_pos*zdot - xdot*z_pos)/(x_pos^2);
    thrust_profile = [thrust_profile Thrust];
    time = [time t];
    gravity_profile = [gravity_profile gravity_force];
end

%% Full Dynamics

function dydt = missile_6dof_dynamics(t, y)
    % Missile 6-DOF dynamics model
    
    % Preallocate output vector
    dydt = zeros(12, 1);

    % Extract states
    x_pos = y(1); y_pos = y(2); z_pos = y(3);
    u = y(4); v = y(5); w = y(6);
    phi = y(7); theta = y(8); psi = y(9);
    P = y(10); Q = y(11); R = y(12);

    % Missile parameters
    m = 205;      % Mass (kg)
    Radius = 7.62e-2; % Average Radius of Missile (m) 
    Length = 1; % Length of Missile (m)
    I = Missile_Inertia_Matrix(m, Radius, Length);  % Inertia matrix (kg*m^2)
    l_ref = 0.5;

    %% Positions
    R_Inertial_to_Body = Rotate(phi, 'x') * Rotate(theta, 'y') * Rotate(psi, 'z');
    dydt(1:3) = R_Inertial_to_Body' * [u; v; w];

    %% Rotations
    R_Eulers = [1 sin(phi) * tan(theta) cos(phi) * tan(theta);
                0 cos(theta) -sin(phi);
                0 sin(phi) / cos(theta) cos(phi) / cos(theta)];
    dydt(4:6) = R_Eulers * [P; Q; R];

    %% Equations of motion

    % Gravity Components
    g = 9.81; % Acceleration of Gravity (m/s^2)
    R_g_to_body = (Rotate(phi, 'x') * Rotate(theta, 'y'));
    weights = R_g_to_body * [0; 0; m * g];

    % Aerodynamic Components
    F_thrust = 0; % compute_thrust(t);
    thrust = [F_thrust; 0; 0];

    [temperature, pressure, density, speedOfSound] = troposphereModel(-z_pos);
    Mach_number = norm([u, v, w]) / speedOfSound;
    q = 1/8 * density * norm([u, v, w]) * pi * (2 * Radius)^2;
    [Coef_Fxyz, Coef_Momxyz] = Aerodynamic_Model(u, v, w);
    forces = -q * Coef_Fxyz;
    moments = q * Coef_Momxyz * l_ref;

    Angular_Matrix = [0 -R Q; R 0 -P; -Q P 0];
    dydt(7:9) = (weights + forces + thrust) / m - Angular_Matrix * [u; v; w];
    dydt(10:12) = I \ (moments - Angular_Matrix * I * [P; Q; R]);

end

%% Thrust Profile Function

function F_thrust = compute_thrust(t)
    thrust_max = 10000; % Maximum thrust (N)
    burnout_time = 0.1; % Time until burnout (s)
    flameoff_start = 30; % Start of flame-off (s)
    flameoff_rate = 50; % Rate of linear decrease during flame-off

    % Simple thrust profile
    if t < burnout_time
        % Linear increase until burnout
        F_thrust = thrust_max * (t / burnout_time);
    elseif t < flameoff_start
        % Thrust is constant after burnout until flame-off starts
        F_thrust = thrust_max;
    else
        % Linear decrease during flame-off
        F_thrust = thrust_max * max(0, 1 - (t - flameoff_start) / flameoff_rate);
    end
end


%% Gravity Model Function

function g = gravity_model(h)
    % International Standard Atmosphere (ISA) gravity model
    g0 = 9.81;  % Standard acceleration due to gravity at sea level (m/s^2)
    R0 = 6371e3;  % Earth's mean radius (m)

    g = g0 * (R0 / (R0 + h))^2;
end

%% Plotting Function

function plot_results(t, state)
    % Plot the simulation results
    figure;
    state(:,1:6) = state(:,1:6)/1000; % km , km/s
    state(:,8:10) = state(:,8:10)/1000; % km
    
    % Translational motion
    subplot(3, 2, 1);
    plot(t, state(:, 1:3));
    title('Positions in Inertial Frame');
    xlabel('Time (s)');
    ylabel('Position (km)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % Set the limits for the first subplot
    limitIncreaseFactor = 0.2;
    newLimits = edit_limits(axis, limitIncreaseFactor);
    axis(newLimits);

    subplot(3, 2, 2)
    plot(t, state(:,8:10));
    title('Velocities Motion in Inertial Frame');
    xlabel('Time (s)');
    ylabel('Velocity (km/s)');
    legend('Xdot', 'Ydot', 'Zdot');
    grid on;
    
    % Set the limits for the second subplot
    newLimits = edit_limits(axis, limitIncreaseFactor);
    axis(newLimits);

    % Rotational motion
    subplot(3, 2, 3);
    plot(t, state(:,4:6));
    title('Velocities in Body-Frame');
    xlabel('Time (s)');
    ylabel('Velocity (km/s)');
    legend('u','v','w');
    grid on;
    
    % Set the limits for the third subplot
    newLimits = edit_limits(axis, limitIncreaseFactor);
    axis(newLimits);

    subplot(3, 2, 4);
    plot(t, rad2deg(state(:,7)));
    title('Angle of Projection');
    xlabel('Time (s)');
    ylabel('Theta (Degrees)');
    grid on;
    
    % Set the limits for the fourth subplot
    newLimits = edit_limits(axis, limitIncreaseFactor);
    axis(newLimits);

    % 3D Path
    subplot(3, 2, 5);
    plot3(state(:, 1), state(:, 2), state(:, 3), '-.', 'LineWidth', 0.8);
    title('3D Path in Inertial Frame');
    xlabel('X (km)');
    ylabel('Y (km)');
    zlabel('Z (km)');
    grid on;
    view(3);
    
    % Set the limits for the fifth subplot


    subplot(3, 2, 6);
    plot(state(:, 1), state(:, 3));
    title('X vs Z (2D-Path projection)');
    xlabel('X (km)');
    ylabel('Z (km)');
    grid on;
    
    % Set the limits for the sixth subplot
    newLimits = edit_limits(axis, limitIncreaseFactor);
    axis(newLimits);

end

function newLimits = edit_limits(axis, limitIncreaseFactor)
    currentLimits = axis;
    evenIndices = 2:2:numel(currentLimits);
    oddIndices = 1:2:numel(currentLimits);
    increase_arr = repelem(currentLimits(evenIndices), 2);
    increase_arr(oddIndices) = -increase_arr(oddIndices);
    newLimits = currentLimits + limitIncreaseFactor * increase_arr;
end