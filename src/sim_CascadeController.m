classdef sim_CascadeController < handle
    % sim_CascadeController: Cascaded position / attitude controller.
    %
    %   position (100 Hz) -> tilt + thrust references
    %   attitude (500 Hz) -> body torques
    %   mixer             -> rotor speeds
    %
    % The control law follows Eq. (23) of Luukkonen (2011), including its
    % inertia normalisation:
    %
    %   tau_phi = (Kd*(phid_dot - phi_dot) + Kp*(phid - phi)) * Ixx
    %   T       = (g + Kd*(zd_dot - z_dot) + Kp*(zd - z)) * m
    %             / (cos(phi)*cos(theta))
    %
    % That normalised form is what makes gains interpretable: for a
    % second-order loop, Kp = wn^2 and Kd = 2*zeta*wn directly, in rad/s
    % and dimensionless damping. Project_2/3 used raw torque gains, where
    % Kp_phi = 34.3 hides a natural frequency of sqrt(34.3/0.004856) =
    % 84 rad/s - about seven times a sensible attitude bandwidth for this
    % airframe, and fast enough that the 125 Hz control rate those
    % projects ran at was itself the binding stability constraint.
    %
    % Gains here are given as (wn, zeta) pairs, so the bandwidth
    % separation between loops is visible by inspection.

    properties
        % Loop bandwidths [rad/s] and damping ratios [-]
        wnAtt = 12.0;   zetaAtt = 0.90;   % roll / pitch
        wnYaw =  5.0;   zetaYaw = 0.90;   % yaw
        wnAlt =  2.5;   zetaAlt = 0.90;   % altitude
        wnPos =  1.5;   zetaPos = 0.90;   % horizontal position

        % Integral gains. The paper's controller is pure PD and leaves a
        % standing altitude error under any thrust bias; a small integral
        % term removes it. Set both to 0 for paper-exact behaviour.
        kiAlt = 0.8;    iLimAlt = 2.0;    % [m s] anti-windup clamp
        kiPos = 0.3;    iLimPos = 2.0;
    end

    properties (SetAccess = private)
        % Attitude references, held between outer-loop updates so the
        % inner loop always has a reference even though it runs five
        % times more often than the loop that produces one.
        refPsi = 0
        refTheta = 0
        refPhi = 0
    end

    properties (Access = private)
        P
        refThrustAccel = 0
        intAlt = 0
        intPos = zeros(2, 1)
    end

    methods
        %% Construction
        function obj = sim_CascadeController(P)
            % Constructor.
            %
            % Input:
            %   P - Parameter struct from sim_droneParams()

            obj.P = P;
        end

        %% Reset
        function reset(obj)
            % reset: Clear integrators and held references.
            obj.intAlt = 0;
            obj.intPos = zeros(2, 1);
            obj.refPsi = 0;
            obj.refTheta = 0;
            obj.refPhi = 0;
            obj.refThrustAccel = 0;
        end

        %% Outer loop: position -> attitude and thrust references
        function outerLoop(obj, x, ref, dt)
            % outerLoop: PID on inertial position, producing tilt
            % references and a vertical acceleration demand.
            %
            % Inputs:
            %     x - Current 12x1 state
            %   ref - Struct with fields pos [3x1], vel [3x1], yaw
            %    dt - Outer loop period [s]

            ePos = ref.pos - x(1:3);
            eVel = ref.vel - x(4:6);

            %% Altitude
            obj.intAlt = sim_saturate(obj.intAlt + ePos(3)*dt, ...
                -obj.iLimAlt, obj.iLimAlt);

            [kp, kd] = sim_CascadeController.pdGains(obj.wnAlt, obj.zetaAlt);
            obj.refThrustAccel = kp*ePos(3) + kd*eVel(3) ...
                + obj.kiAlt*obj.intAlt;

            %% Horizontal position
            obj.intPos = sim_saturate(obj.intPos + ePos(1:2)*dt, ...
                -obj.iLimPos, obj.iLimPos);

            [kp, kd] = sim_CascadeController.pdGains(obj.wnPos, obj.zetaPos);
            aInertial = kp*ePos(1:2) + kd*eVel(1:2) ...
                + obj.kiPos*obj.intPos;

            %% Inertial acceleration -> tilt references
            % Rotate the demanded acceleration into the heading frame,
            % then invert the small-angle thrust map
            %   xddot ~=  g*theta  ->  theta_ref =  ax_body/g
            %   yddot ~= -g*phi    ->  phi_ref   = -ay_body/g
            %
            % Project_2/3 rotated the position ERROR and then applied
            % the gains, which is equivalent only while Kp_x == Kp_y.
            % Rotating the acceleration stays correct if the axes are
            % ever tuned apart.
            psi = x(7);
            cPsi = cos(psi); sPsi = sin(psi);

            axBody =  cPsi*aInertial(1) + sPsi*aInertial(2);
            ayBody = -sPsi*aInertial(1) + cPsi*aInertial(2);

            % Rate-limit as well as clamp. A step in tilt reference is a
            % step the inner loop cannot track, and asking for one only
            % builds overshoot.
            maxStep = obj.P.maxTiltRate * dt;

            thetaRef = sim_saturate( axBody/obj.P.g, ...
                obj.refTheta - maxStep, obj.refTheta + maxStep);
            phiRef   = sim_saturate(-ayBody/obj.P.g, ...
                obj.refPhi - maxStep, obj.refPhi + maxStep);

            obj.refTheta = sim_saturate(thetaRef, ...
                -obj.P.maxTilt, obj.P.maxTilt);
            obj.refPhi   = sim_saturate(phiRef, ...
                -obj.P.maxTilt, obj.P.maxTilt);
            obj.refPsi   = ref.yaw;
        end

        %% Manual references, bypassing the position loop
        function manualLoop(obj, cmd, x, dt)
            % manualLoop: Turn pilot stick input into the same attitude
            % and thrust references the outer loop produces.
            %
            % This is "angle mode" on a real flight controller: the
            % sticks command a tilt ANGLE and a climb RATE, and the
            % inner loop still stabilises the airframe. Project_2's
            % manualControl mapped the sticks straight onto motor speeds
            % open-loop, which is not flyable by a human on a keyboard.
            %
            % Inputs:
            %   cmd - Command struct from sim_PilotInput: roll, pitch, yawRate,
            %         throttle, each in [-1, 1]
            %     x - Current 12x1 state
            %    dt - Period of this update [s]

            obj.refPhi   = cmd.roll  * obj.P.maxTilt;
            obj.refTheta = cmd.pitch * obj.P.maxTilt;

            % Yaw stick is a rate command, integrated into a heading
            % reference so the drone holds heading when centred.
            obj.refPsi = sim_wrapAngle(obj.refPsi ...
                + cmd.yawRate * obj.P.maxYawRate * dt);

            % Throttle is a climb-rate command closed onto measured
            % vertical velocity, so releasing the stick holds altitude
            % instead of dropping to whatever thrust the stick implies.
            [~, kd] = sim_CascadeController.pdGains(obj.wnAlt, obj.zetaAlt);
            obj.refThrustAccel = kd * ...
                (cmd.throttle*obj.P.maxClimbRate - x(6));

            % The position integrators are not running in manual mode;
            % clear them so switching back to auto starts clean.
            obj.intPos = zeros(2, 1);
            obj.intAlt = 0;
        end

        %% Inner loop: attitude -> body torques -> wrench
        function u = innerLoop(obj, drone)
            % innerLoop: PD on Euler angles, Eq. (23), returning the
            % wrench the mixer should try to produce.
            %
            % Input:
            %   drone - sim_QuadrotorPlant whose state and Euler rates are read
            %
            % Output:
            %       u - 4x1 [T; tau_phi; tau_theta; tau_psi]

            x = drone.xState;

            % Derivative term on Euler rates, not raw body rates.
            etaDot = drone.eulerRates();

            % All three errors wrapped: yaw obviously, but roll and pitch
            % too, so a reference near the boundary never produces a
            % 2*pi error spike.
            ePsi   = sim_wrapAngle(obj.refPsi   - x(7));
            eTheta = sim_wrapAngle(obj.refTheta - x(8));
            ePhi   = sim_wrapAngle(obj.refPhi   - x(9));

            [kpA, kdA] = sim_CascadeController.pdGains(obj.wnAtt, obj.zetaAtt);
            [kpY, kdY] = sim_CascadeController.pdGains(obj.wnYaw, obj.zetaYaw);

            % Inertia normalisation, Eq. (23): the bracket is an angular
            % acceleration, multiplying by I gives a torque.
            tauPhi   = obj.P.Ix * (kpA*ePhi   - kdA*etaDot(3));
            tauTheta = obj.P.Iy * (kpA*eTheta - kdA*etaDot(2));
            tauPsi   = obj.P.Iz * (kpY*ePsi   - kdY*etaDot(1));

            %% Thrust with gravity compensation, Eq. (23)
            % T*cos(phi)*cos(theta) must carry the weight, so the tilt
            % division restores the vertical component. Guard the
            % divisor: past 60 deg of tilt the correction asks for thrust
            % the drone does not have, and letting it run to infinity
            % only saturates the mixer in a less informative way.
            tiltCos = max(cos(x(9)) * cos(x(8)), 0.5);
            T = obj.P.m * (obj.P.g + obj.refThrustAccel) / tiltCos;

            u = [T; tauPhi; tauTheta; tauPsi];
        end
    end

    methods (Static)
        %% Second-order gain conversion
        function [kp, kd] = pdGains(wn, zeta)
            % pdGains: Normalised PD gains for a target second-order
            % response. Closing an inertia-normalised loop with these
            % gives  edd + 2*zeta*wn*ed + wn^2*e = 0.
            %
            % Inputs:
            %     wn - Natural frequency [rad/s]
            %   zeta - Damping ratio [-]
            %
            % Outputs:
            %     kp - Proportional gain
            %     kd - Derivative gain

            kp = wn^2;
            kd = 2 * zeta * wn;
        end
    end
end
