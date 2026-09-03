function a = sim_wrapAngle(a)
    % sim_wrapAngle: Wrap angles to the range [-pi, pi].
    %
    % Replaces wrapToPi, which lives in the Mapping Toolbox. Project_2/3
    % called wrapToPi on the hot path, so the simulation refused to start
    % without a toolbox nothing else in the project needs.
    %
    % Input:
    %   a - Angle or array of angles [rad]
    %
    % Output:
    %   a - Equivalent angles wrapped to [-pi, pi]

    a = mod(a + pi, 2*pi) - pi;
end
