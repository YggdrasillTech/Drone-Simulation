function P = sim_droneParams(layout)
    % sim_droneParams: Physical parameters of the quadcopter.
    %
    % Every value traceable to the reference paper is tagged with the
    % equation or table it comes from:
    %
    %   Teppo Luukkonen, "Modelling and control of quadcopter",
    %   Independent research project in applied mathematics,
    %   Aalto University, Espoo, August 2011.
    %
    % Values NOT in the paper (motor limits, actuator lag) are tagged
    % [ENG] and carry the engineering justification inline, so the
    % provenance of the model is never ambiguous.
    %
    % Inputs:
    %   layout - Rotor geometry, one of:
    %              'plus' - Paper layout (Fig. 1): rotors on the body
    %                       x/y axes. Reproduces Eq. (8) exactly.
    %              'x'    - 45 deg layout: rotors on the diagonals.
    %                       Matches the STL airframe used for the
    %                       animation. Default.
    %
    % Output:
    %   P - Struct of parameters consumed by robotDrone / droneMixer.

    arguments (Input)
        layout (1,:) char = 'x'
    end

    %% Environment
    P.g = 9.81;             % [m/s^2]   Table 1

    %% Rigid body
    P.m = 0.468;            % [kg]      Table 1
    P.l = 0.225;            % [m]       Table 1, rotor to centre of mass

    % Diagonal inertia matrix, Eq. (5). Ixx = Iyy by symmetry.
    P.Ix = 4.856e-3;        % [kg m^2]  Table 1
    P.Iy = 4.856e-3;        % [kg m^2]  Table 1
    P.Iz = 8.801e-3;        % [kg m^2]  Table 1

    %% Rotors
    % f_i = k*w_i^2, tau_Mi = b*w_i^2 + Ir*wdot_i          Eq. (6)
    P.k  = 2.980e-6;        % [N s^2]     Table 1, lift constant
    P.b  = 1.140e-7;        % [N m s^2]   Table 1, drag constant
    P.Ir = 3.357e-5;        % [kg m^2]    Table 1 (called I_M in paper)

    %% Aerodynamic drag
    % Enters the translational dynamics as -(1/m)*A*xi_dot, Eq. (21).
    % NOTE the 1/m: A is a force-per-velocity coefficient in [kg/s],
    % not an acceleration-per-velocity coefficient in [1/s].
    P.Ax = 0.25;            % [kg/s]    Table 1
    P.Ay = 0.25;            % [kg/s]    Table 1
    P.Az = 0.25;            % [kg/s]    Table 1

    %% Hover operating point (derived, not free)
    % Four rotors must carry the weight: 4*k*wHover^2 = m*g
    P.wHover = sqrt(P.m * P.g / (4 * P.k));   % 620.6108 rad/s

    %% Motor limits                                            [ENG]
    % The paper models no actuator limit. A finite ceiling is required
    % for any realistic simulation, otherwise the mixer can command
    % unbounded thrust and the controller looks better than it is.
    %
    % wMax = 950 rad/s gives a static thrust-to-weight ratio of
    %   4*k*wMax^2 / (m*g) = 2.34
    % which is the usual design point for a stable camera platform of
    % this class (a racing airframe would sit near 5-8). Project_2 used
    % 1500 rad/s (T/W = 5.8), which is not consistent with the 0.468 kg
    % / 0.225 m airframe the paper describes, so 950 is adopted here.
    P.wMax = 950;           % [rad/s]

    % Idle speed: brushless motors are never commanded to a full stop
    % in flight, both to keep control authority and to avoid desync.
    % 15% of hover is a typical idle.
    P.wMin = 0.15 * P.wHover;   % [rad/s]

    % First-order motor + propeller response, wdot = (wCmd - w)/tau.
    % ESC/motor/prop time constants for this class sit at 20-50 ms;
    % 30 ms is a conservative middle. Set to 0 to model an ideal
    % actuator (paper behaviour).
    P.motorTau = 0.030;     % [s]

    %% Rotor geometry
    % The allocation matrix is BUILT from this geometry rather than
    % hard-coded, which is what keeps the yaw torque signs and the
    % gyroscopic term wGamma from drifting out of agreement.
    %
    %   rotorPos  - 2xN, [x; y] offset of each rotor in the body frame
    %   spinSign  - 1xN, +1 / -1 giving the coefficient of each rotor
    %               in tau_psi = b * sum(spinSign_i * w_i^2), and
    %               likewise in wGamma = sum(spinSign_i * w_i).
    %
    % Adjacent rotors must counter-rotate, so spinSign alternates
    % around the airframe.
    switch lower(layout)
        case 'plus'
            % Paper Fig. 1: rotor 1 at +x, 2 at -y, 3 at -x, 4 at +y.
            % Recovers Eq. (8) term for term:
            %   tau_phi   = l*k*(-w2^2 + w4^2)
            %   tau_theta = l*k*(-w1^2 + w3^2)
            %   tau_psi   = b*( w1^2 - w2^2 + w3^2 - w4^2)
            P.rotorPos = P.l * [ 1,  0, -1,  0;
                                 0, -1,  0,  1];
            P.spinSign = [1, -1, 1, -1];

        case 'x'
            % Rotors on the 45 deg diagonals, each still at distance l
            % from the centre of mass. The roll/pitch moment arm is
            % therefore l/sqrt(2) per axis, NOT l.
            %
            % Project_2/3 used a full l on both axes, which silently
            % places the rotors at l*sqrt(2) = 0.318 m while keeping
            % the inertia of a 0.225 m airframe - a 41% overestimate of
            % the available roll/pitch torque.
            c = P.l / sqrt(2);
            P.rotorPos = [ c, -c, -c,  c;
                          -c, -c,  c,  c];
            P.spinSign = [-1, 1, -1, 1];

        otherwise
            error('sim_droneParams:layout', ...
                'Unknown layout "%s". Use ''plus'' or ''x''.', layout);
    end
    P.layout = lower(layout);

    %% Flight envelope used by the controllers                 [ENG]
    P.maxTilt      = deg2rad(25);   % [rad]   roll/pitch reference cap
    P.maxClimbRate = 2.5;           % [m/s]   manual throttle authority
    P.maxYawRate   = deg2rad(120);  % [rad/s] manual yaw authority
    P.maxTiltRate  = deg2rad(180);  % [rad/s] tilt reference slew cap
end
