classdef sim_QuadrotorPlant < handle
    % sim_QuadrotorPlant: The aircraft - rigid-body dynamics, mixer, actuators.
    %
    % Implements the Newton-Euler model of
    %   Luukkonen (2011), "Modelling and control of quadcopter",
    % specifically Eq. (21) for translation, Eq. (11) for rotation and
    % Eq. (4) for the Euler-rate kinematics.
    %
    % STATE VECTOR (unchanged from Project_2/3, so old notes and logs
    % stay valid):
    %
    %    1: 3  x, y, z            inertial position       [m]
    %    4: 6  xdot, ydot, zdot   inertial velocity       [m/s]
    %    7: 9  psi, theta, phi    yaw, pitch, roll        [rad]
    %   10:12  p, q, r            body angular rates      [rad/s]
    %
    % Note the trap this encodes: the ANGLES run yaw-pitch-roll while the
    % RATES run roll-pitch-yaw, so x(7) and x(12) are the same axis, and
    % x(9) and x(10) are the same axis.
    %
    % The allocation matrix is DERIVED from the rotor geometry rather
    % than written out by hand:
    %
    %   [T; tau_phi; tau_theta; tau_psi] = F * [w1^2; ...; w4^2]
    %
    %   row 1 (T)         :  k          every rotor            Eq. (7)
    %   row 2 (tau_phi)   :  k * y_i    torque about x_B
    %   row 3 (tau_theta) : -k * x_i    torque about y_B
    %   row 4 (tau_psi)   :  b * s_i    s = spin direction     Eq. (8)
    %
    % and the gyroscopic rotor term uses the SAME spin vector,
    %
    %   wGamma = sum(s_i * w_i)                                Eq. (11)
    %
    % which is the point of deriving it. Project_3 hard-coded tau_psi
    % with signs [-b +b -b +b] but computed wGamma = w1 - w2 + w3 - w4,
    % i.e. [+1 -1 +1 -1]. The two disagreed, so the gyroscopic coupling
    % pushed roll and pitch the wrong way. Building both from one
    % geometry makes that class of bug unrepresentable.

    properties
        xState              % 12x1 state vector
        omegaMotor          % 4x1 actual rotor speeds [rad/s]
    end

    properties (SetAccess = immutable)
        P                   % Parameter struct from sim_droneParams()
        F                   % 4x4 allocation matrix, wrench = F * w.^2
        Finv                % Precomputed inverse
        spinSign            % 1x4 rotor spin directions (+1 / -1)
    end

    properties (Access = private)
        dragVec             % [Ax; Ay; Az]/m, the acceleration form
        invI                % [1/Ix; 1/Iy; 1/Iz]
        inertiaTerm         % Centripetal coefficients
        sqMin, sqMax        % Squared rotor speed limits
    end

    methods
        %% Construction
        function obj = sim_QuadrotorPlant(P, x0)
            % Constructor.
            %
            % Inputs:
            %    P - Parameter struct from sim_droneParams()
            %   x0 - Initial 12x1 state vector

            arguments (Input)
                P  struct
                x0 (12,1) double
            end

            obj.P = P;

            x = P.rotorPos(1, :);
            y = P.rotorPos(2, :);

            obj.F = [ P.k * ones(1, 4);
                      P.k * y;
                     -P.k * x;
                      P.b * P.spinSign ];

            % F is a constant, well-conditioned 4x4. Project_2/3 solved
            % F\u afresh on every control step of every drone.
            obj.Finv     = inv(obj.F);
            obj.spinSign = P.spinSign(:).';

            obj.xState     = x0;
            obj.omegaMotor = ones(4, 1) * P.wHover;

            % Constant for the life of the object. Recomputing these
            % inside modelDynamics - four times per RK4 step, 500 times a
            % second, per drone - was a measurable share of frame time.
            obj.dragVec     = [P.Ax; P.Ay; P.Az] / P.m;
            obj.invI        = [1/P.Ix; 1/P.Iy; 1/P.Iz];
            obj.inertiaTerm = [(P.Iy - P.Iz) / P.Ix;
                               (P.Iz - P.Ix) / P.Iy;
                               (P.Ix - P.Iy) / P.Iz];
            obj.sqMin = P.wMin^2;
            obj.sqMax = P.wMax^2;
        end

        %% One full simulation step
        function u = advance(obj, t, dt, uDesired, groundLevel)
            % advance: Actuators, integration, wrapping and ground, in
            % the order the simulation loop needs them.
            %
            % Collapsing these into one call is what keeps the loop
            % bodies of RunSingleDrone and RunSwarm short enough
            % to read, and guarantees both do it identically.
            %
            % Inputs:
            %            t - Current time [s]
            %           dt - Step size [s]
            %     uDesired - 4x1 wrench the controller asked for
            %  groundLevel - Floor height [m], -Inf to disable
            %
            % Output:
            %            u - 4x1 wrench actually applied

            u = obj.applyMotorCommand(uDesired, dt);

            obj.xState = sim_rk4Step(@(tt, xx) obj.modelDynamics(tt, xx, u), ...
                t, dt, obj.xState);

            % Normalize Angles to the range [-pi, pi]
            obj.xState(7:9) = sim_wrapAngle(obj.xState(7:9));

            % The paper's model has no ground, so a drone commanded
            % downwards falls through it. A perfectly inelastic floor is
            % enough to keep manual flight sane. Downward motion only: a
            % climbing drone at the floor is taking off, not colliding.
            if obj.xState(3) < groundLevel
                obj.xState(3) = groundLevel;
                obj.xState(6) = max(obj.xState(6), 0);
            end
        end

        %% Dynamics
        function dx = modelDynamics(obj, ~, x, u)
            % modelDynamics: State derivative of the quadcopter.
            %
            % Inputs:
            %   t - Current time (the model is time-invariant)
            %   x - 12x1 state vector, layout in the class header
            %   u - Control input vector:
            %         u(1) = Total thrust T
            %         u(2) = Roll torque tau_phi
            %         u(3) = Pitch torque tau_theta
            %         u(4) = Yaw torque tau_psi
            %
            % Output:
            %   dx - State derivative vector

            dx = zeros(12, 1);

            psi = x(7);  theta = x(8);  phi = x(9);
            p   = x(10); q     = x(11); r   = x(12);

            % Trigonometry dominates the cost of this function and every
            % term is needed more than once, so evaluate each once.
            sPsi = sin(psi);   cPsi = cos(psi);
            sThe = sin(theta); cThe = cos(theta);
            sPhi = sin(phi);   cPhi = cos(phi);

            % The Euler-rate transform of Eq. (4) is singular at
            % theta = +-pi/2. Clamp cos(theta) away from zero so a
            % transient through vertical degrades instead of returning
            % Inf and poisoning the whole state vector.
            if abs(cThe) < 1e-4
                cThe = sign(cThe + (cThe == 0)) * 1e-4;
            end

            %% Position derivatives
            dx(1) = x(4);
            dx(2) = x(5);
            dx(3) = x(6);

            %% Linear accelerations, Eq. (21)
            % The third column of the rotation matrix R rotates the
            % body-z thrust into the inertial frame. Note the drag term
            % is divided by mass: A is in kg/s, so A/m is the
            % acceleration-per-velocity coefficient. Project_2/3 wrote
            % "- obj.Ax*x(4)", making drag too weak by a factor of m.
            Tm = u(1) / obj.P.m;

            dx(4) = Tm*(cPsi*sThe*cPhi + sPsi*sPhi) - obj.dragVec(1)*x(4);
            dx(5) = Tm*(sPsi*sThe*cPhi - cPsi*sPhi) - obj.dragVec(2)*x(5);
            dx(6) = Tm*(cThe*cPhi) - obj.P.g        - obj.dragVec(3)*x(6);

            %% Euler angle derivatives, Eq. (4)
            coupled = sPhi*q + cPhi*r;
            dx(7) = coupled / cThe;                      % yaw   psi
            dx(8) = cPhi*q - sPhi*r;                     % pitch theta
            dx(9) = p + (sThe/cThe)*coupled;             % roll  phi

            %% Angular accelerations, Eq. (11)
            % Relative rotor speed for the gyroscopic term, taken from
            % the same spin vector that builds tau_psi.
            gyro = obj.P.Ir * (obj.spinSign * obj.omegaMotor);

            % p_dot = ((Iy-Iz)/Ix)*q*r - (Ir/Ix)*q*wGamma + tau_phi/Ix
            dx(10) = obj.inertiaTerm(1)*q*r ...
                - obj.invI(1)*gyro*q + obj.invI(1)*u(2);

            % q_dot = ((Iz-Ix)/Iy)*p*r + (Ir/Iy)*p*wGamma + tau_theta/Iy
            dx(11) = obj.inertiaTerm(2)*p*r ...
                + obj.invI(2)*gyro*p + obj.invI(2)*u(3);

            % r_dot has no gyroscopic term: the rotor angular momentum
            % is already along the body z-axis.
            dx(12) = obj.inertiaTerm(3)*p*q + obj.invI(3)*u(4);
        end

        %% Euler rates from body rates
        function etaDot = eulerRates(obj)
            % eulerRates: Body rates through W_eta^-1, Eq. (4).
            %
            % The attitude controller compares against Euler-angle
            % references, so its derivative term belongs on Euler rates.
            % Project_2/3 fed the raw body rates p, q, r into a PD loop
            % on psi, theta, phi. Those agree only near hover; at the
            % 30 deg initial roll the projects actually use, they do not.
            %
            % Output:
            %   etaDot - 3x1 [psiDot; thetaDot; phiDot] [rad/s]

            x = obj.xState;
            theta = x(8); phi = x(9);

            cThe = cos(theta);
            if abs(cThe) < 1e-4
                cThe = sign(cThe + (cThe == 0)) * 1e-4;
            end

            sPhi = sin(phi); cPhi = cos(phi);
            coupled = sPhi*x(11) + cPhi*x(12);

            etaDot = [ coupled / cThe;
                       cPhi*x(11) - sPhi*x(12);
                       x(10) + (sin(theta)/cThe)*coupled ];
        end

        %% Actuators
        function u = applyMotorCommand(obj, uDesired, dt)
            % applyMotorCommand: Push a desired wrench through the mixer
            % and the motor lag, returning what is really applied.
            %
            % The paper treats rotor speeds as the control input, i.e.
            % an ideal actuator. Real ESC/motor/prop combinations answer
            % with a first-order lag of a few tens of milliseconds,
            % close enough to the attitude bandwidth to matter. Set
            % P.motorTau = 0 to recover the paper's behaviour.
            %
            % Inputs:
            %   uDesired - 4x1 wrench requested by the controller
            %         dt - Time step [s]
            %
            % Output:
            %          u - 4x1 wrench actually produced

            omegaCmd = obj.speedsFromWrench(uDesired);

            if obj.P.motorTau > 0
                % Exact discrete solution of the first-order lag, which
                % is stable for any dt - unlike the explicit form, which
                % diverges once dt > 2*tau.
                alpha = 1 - exp(-dt / obj.P.motorTau);
                obj.omegaMotor = obj.omegaMotor ...
                    + alpha * (omegaCmd - obj.omegaMotor);
            else
                obj.omegaMotor = omegaCmd;
            end

            u = obj.wrenchFromSpeeds(obj.omegaMotor);
        end

        %% Mixer, forward
        function u = wrenchFromSpeeds(obj, omega)
            % wrenchFromSpeeds: rotor speeds -> [T; tau_phi; tau_theta;
            % tau_psi].
            u = obj.F * (omega(:).^2);
        end

        %% Mixer, inverse
        function [omega, uReal] = speedsFromWrench(obj, u)
            % speedsFromWrench: Inverse map with prioritised
            % desaturation.
            %
            % Naively inverting and then clipping - what Project_2/3 did
            % - silently destroys the torque the attitude loop asked for
            % exactly when attitude control matters most. Each rotor
            % clips by a different amount, so the moment that comes out
            % points somewhere other than the one requested.
            %
            % The demand is instead fitted into the rotor envelope in
            % order of how much it matters to staying in the air:
            %
            %   1. collective thrust  - without it nothing else helps
            %   2. roll and pitch     - these hold the aircraft up
            %   3. yaw                - heading is merely desirable
            %
            % Roll and pitch are scaled by one common factor so their
            % direction survives exactly; yaw gets what is left. That
            % order matters here: this airframe can only make about
            % 0.09 N m of yaw torque at hover thrust, so a large heading
            % error saturates yaw routinely, and scaling all three
            % together would let that steal roll and pitch authority.
            %
            % Input:
            %       u - 4x1 desired [T; tau_phi; tau_theta; tau_psi]
            %
            % Outputs:
            %   omega - 4x1 achievable rotor speeds [rad/s]
            %   uReal - 4x1 wrench those speeds actually produce

            lo = obj.sqMin;
            hi = obj.sqMax;

            sqT  = obj.Finv * [u(1); 0;    0;    0];
            sqRP = obj.Finv * [0;    u(2); u(3); 0];
            sqY  = obj.Finv * [0;    0;    0;    u(4)];

            % 1. Collective. A pure-thrust command spreads equally over
            %    the rotors, so shifting it to fit is torque-neutral.
            headroom = min(hi - sqT);
            deficit  = min(sqT - lo);
            if headroom < 0
                sqT = sqT + headroom;
            elseif deficit < 0
                sqT = sqT - deficit;
            end
            base = sim_saturate(sqT, lo, hi);

            % 2. Roll and pitch, direction preserved.
            base = base + sim_QuadrotorPlant.fitScale(base, sqRP, lo, hi) * sqRP;

            % 3. Yaw, with whatever authority is left.
            base = base + sim_QuadrotorPlant.fitScale(base, sqY, lo, hi) * sqY;

            omegaSq = sim_saturate(base, lo, hi);
            omega   = sqrt(omegaSq);

            if nargout > 1
                uReal = obj.F * omegaSq;
            end
        end
    end

    methods (Static, Access = private)
        %% Largest feasible fraction of a differential demand
        function s = fitScale(base, delta, lo, hi)
            % fitScale: Biggest s in [0, 1] with base + s*delta inside
            % [lo, hi] on every rotor. Rotors with no share of this
            % demand cannot constrain the scale, and must not be
            % divided by.

            s = 1;

            for i = 1:4
                if delta(i) > 0 && base(i) + delta(i) > hi
                    s = min(s, (hi - base(i)) / delta(i));
                elseif delta(i) < 0 && base(i) + delta(i) < lo
                    s = min(s, (lo - base(i)) / delta(i));
                end
            end

            s = sim_saturate(s, 0, 1);
        end
    end
end
