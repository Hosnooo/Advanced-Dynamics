%% Atmospheric Model Function

function [temperature, pressure, density, speedOfSound] = troposphereModel(altitude)
    % U.S. Standard Atmosphere model for calculating atmospheric properties
    % Constants for the U.S. Standard Atmosphere model
    h0 = 0;             % Base altitude (m)
    T0 = 288.15;        % Sea-level temperature (K)
    P0 = 101325;        % Sea-level pressure (Pa)
    L = 0.0065;         % Temperature lapse rate (K/m)
    R = 287.05;         % Specific gas constant (J/(kg·K))
    g0 = 9.80665;       % Standard acceleration of gravity (m/s^2)
    gamma = 1.4;        % Standard heat capacity ratio of Air
    
    % Calculate temperature, pressure, and density based on altitude
    temperature = T0 - L * (altitude - h0);
    pressure = P0 * (temperature / T0) .^ (g0 / (L * R));
    density = P0 ./ (R * temperature);
    
    % Calculate speed of sound based on temperature
    speedOfSound = sqrt(gamma * R * temperature);
end