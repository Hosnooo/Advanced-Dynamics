function F_thrust = compute_thrust(t)
    thrust_max = 10e3;
    burnout_time = 0.1; % Time until burnout (s)
    flameoff_start = 40; % Start of flame-off (s)
    flameoff_rate = 40; % Rate of linear decrease during flame-off

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