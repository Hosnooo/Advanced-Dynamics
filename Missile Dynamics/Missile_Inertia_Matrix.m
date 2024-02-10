%% Missile Inertia Function
function I = Missile_Inertia_Matrix(m, R, L)
    % Compute inertia matrix for the missile
    Ixx = 0.5 * m * R^2; Iyy = 1/12 * m * (3 * R^2 + L^2); Izz = Iyy;
    Ixy = 0; Iyz = 0; Ixz = 0;
    I = [Ixx, -Ixy, -Ixz;
         -Ixy, Iyy, -Iyz;
         -Ixz, -Iyz, Izz];
end