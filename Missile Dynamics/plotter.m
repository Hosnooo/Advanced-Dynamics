load("Aerodynamic_skid.mat")
plot_results(t, y)

% thrust_profile_plot = [thrust_profile(:,1:50-1), thrust_profile(:,51+1:end)];
% thrust_profile_plot = thrust_profile_plot(:,1:length(t));
thrust_profile_plot = thrust_profile;
figure()
subplot(3,1,1)
x = [0:length(thrust_profile_plot)-1]/length(thrust_profile_plot)*60;
plot([0:length(thrust_profile_plot)-1]/length(thrust_profile_plot)*t(end),...
    thrust_profile_plot'/1000,'-.','LineWidth', 1.2)
grid on;
xlabel('time (sec)')
ylabel('Force (Kilo-Newton)')
title('Thrust Profile')
limitIncreaseFactor = 0.2;
newLimits = edit_limits(axis, limitIncreaseFactor);
    axis(newLimits);

subplot(3,1,2)
plot([0:length(packet1)-1]/length(packet1)*t(end),packet1'/1000,'LineWidth', 1.2)
grid on;
xlabel('time (sec)')
ylabel('Force (Kilo-Newton)')
title('Aerodynamic Forces')
legend('Fx', 'Fy', 'Fz')
limitIncreaseFactor = 0.2;
newLimits = edit_limits(axis, limitIncreaseFactor);
    axis(newLimits);

subplot(3,1,3)
plot([0:length(packet1)-1]/length(packet1)*t(end),packet2','LineWidth', 1.2)
grid on;
xlabel('time (sec)')
ylabel('Moment (N.m)')
title('Aerodynamic Moments')
legend('L', 'M', 'N')

limitIncreaseFactor = 0.2;
newLimits = edit_limits(axis, limitIncreaseFactor);
    axis(newLimits);

    %% Plotting Function
function plot_results(t, state)
    % Plot the simulation results
    figure;
    Velocities = state(:,1:3)/1000;
    Positions = state(:,4:6)/1000;
    Velocities_Inertial = state(:,13:15)/1000;
    Eulers = state(:,10:12);

    % Translational motion
    subplot(3, 2, 1);
    plot(t, Positions);
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
    plot(t, Velocities_Inertial);
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
    plot(t, Velocities);
    title('Velocities in Body-Frame');
    xlabel('Time (s)');
    ylabel('Velocity (km/s)');
    legend('u','v','w');
    grid on;
    
    % Set the limits for the third subplot
    newLimits = edit_limits(axis, limitIncreaseFactor);
    axis(newLimits);

    subplot(3, 2, 4);
    plot(t, rad2deg(Eulers));
    title('Euler Angles');
    xlabel('Time (s)');
    ylabel('Angle (Degrees)');
    grid on;
    legend('Psi', 'Theta', 'Phi')

    % Set the limits for the fourth subplot
    newLimits = edit_limits(axis, limitIncreaseFactor);
    axis(newLimits);

    % 3D Path
    subplot(3, 2, [5 6]);
    plot3(Positions(:, 1), Positions(:, 2), Positions(:, 3), '-.', 'LineWidth', 0.8);
    title('3D Path in Inertial Frame');
    xlabel('X (km)');
    ylabel('Y (km)');
    zlabel('Z (km)');
    grid on;
    view(3);
    
    % Set the limits for the fifth subplot


    % subplot(3, 2, 6);
    % plot(Positions(:, 1), Positions(:, 3));
    % title('X vs Z (2D-Path projection)');
    % xlabel('X (km)');
    % ylabel('Z (km)');
    % grid on;
    % 
    % % Set the limits for the sixth subplot
    % newLimits = edit_limits(axis, limitIncreaseFactor);
    % axis(newLimits);

end

function newLimits = edit_limits(axis, limitIncreaseFactor)
    currentLimits = axis;
    evenIndices = 2:2:numel(currentLimits);
    oddIndices = 1:2:numel(currentLimits);
    increase_arr = repelem(currentLimits(evenIndices), 2);
    increase_arr(oddIndices) = -increase_arr(oddIndices);
    newLimits = currentLimits + limitIncreaseFactor * increase_arr;
end