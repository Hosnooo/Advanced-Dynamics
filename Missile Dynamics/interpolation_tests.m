clc; clear;
% Example usage
mach_vec = [0.6, 0.7, 0.8, 0.9, 1.0];
alpha_vec = [-5, 0, 5, 10, 15, 20];

% Example lookup table (replace this with your actual data)
coeffTable = [
    0.1, 0.2, 0.3, 0.4, 0.5;
    0.2, 0.3, 0.4, 0.5, 0.6;
    0.3, 0.4, 0.5, 0.6, 0.7;
    0.4, 0.5, 0.6, 0.7, 0.8;
    0.5, 0.6, 0.7, 0.8, 0.9
    0.8, 0.9, 0.98, 1, 1.9
];

% Example Mach number and angle of attack
mach = 0.1;
alpha = 3;

