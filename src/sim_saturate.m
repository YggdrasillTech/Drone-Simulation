function y = sim_saturate(x, lo, hi)
    % sim_saturate: Clamp x elementwise to the interval [lo, hi].
    %
    % Inputs:
    %    x - Value or array to clamp
    %   lo - Lower bound (scalar or same size as x)
    %   hi - Upper bound (scalar or same size as x)
    %
    % Output:
    %    y - Clamped value

    y = min(max(x, lo), hi);
end
