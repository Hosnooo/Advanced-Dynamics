function coefficient = aerodynamicCoefficientInterpolation(mach_vec, alpha_vec, coeffTable, mach, alpha)
    % mach_vec: Vector of Mach numbers
    % alpha_vec: Vector of angles of attack
    % coeffTable: 2D lookup table for aerodynamic coefficient
    % mach: Mach number for interpolation/extrapolation
    % alpha: Angle of attack for interpolation/extrapolation

    % Find indices and distances for interpolation/extrapolation
    [machIndices, machDistances, machFractions] = preLookup(mach, mach_vec);
    [alphaIndices, alphaDistances, alphaFractions] = preLookup(alpha, alpha_vec);

    % Perform 2D interpolation using the indices and fractions
    coefficient = interp2(mach_vec, alpha_vec, coeffTable, ...
                         mach + machDistances * machFractions, ...
                         alpha + alphaDistances * alphaFractions, 'linear', 0);

    % Handle extrapolation cases
    if isempty(coefficient)
        % Extrapolation logic using distances and fractions
         extrapolatedValue = extrapolate2D(mach, alpha, mach_vec, alpha_vec, lookupTable);
                                       
        coefficient = extrapValue;
    end
end


function [indices, distances, fractions] = preLookup(value, vector)
    % value: Value for which to perform pre-lookup
    % vector: Vector of grid points in a dimension

    % Find the lower grid point and its index
    lowerGridPoint = find(vector <= value, 1, 'last');

    % If the value is less than the smallest grid point, set to the smallest grid point
    if isempty(lowerGridPoint)
        lowerGridPoint = 1;
    end

    % If the value is greater than the largest grid point, set to the largest grid point
    if lowerGridPoint == numel(vector)
        lowerGridPoint = lowerGridPoint - 1;
    end

    % Find the upper grid point
    upperGridPoint = lowerGridPoint + 1;

    % Calculate the distance and fraction
    distance = value - vector(lowerGridPoint);
    totalDistance = vector(upperGridPoint) - vector(lowerGridPoint);
    fraction = distance / totalDistance;

    % Store the results
    indices = [lowerGridPoint, upperGridPoint];
    distances = distance;
    fractions = fraction;
end

function extrapolatedValue = extrapolate2D(mach, alpha, mach_vec, alpha_vec, lookupTable)
    % Check if values are within the range
    if mach < min(mach_vec) || mach > max(mach_vec) || alpha < min(alpha_vec) || alpha > max(alpha_vec)
        % Values are out of range, perform extrapolation using the nearest available data
        mach = min(max(mach, min(mach_vec)), max(mach_vec));
        alpha = min(max(alpha, min(alpha_vec)), max(alpha_vec));
        
        extrapolatedValue = interp2(mach_vec, alpha_vec, lookupTable, mach, alpha, 'linear', 0);
        return;
    end
end