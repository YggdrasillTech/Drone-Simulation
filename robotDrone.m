classdef robotDrone < handle
    properties
        % State vector
        xState   
        % Motors Speed
        omegaMotor
        maxOmega = 950;
        % Drone internal parameters (defined at creation).
        g  = 9.81;      % [m/s^2]
        m  = 0.468;     % [kg]
        l  = 0.225;     % [m]
        wn = 620.610762 % [rad/s] Nominal motor speed
        k  = 2.98e-6;   % Motor Speed to Force Constant
        b  = 1.14e-7;   % Motor Speed to Reactive Torque Constant

        % Moment of Inertia Constants.
        Ix = 4.856e-3;
        Iy = 4.856e-3;
        Iz = 8.801e-3;
        
        % Motor Inertia for Gyroscopic effects
        Ir = 3.357e-5;

        % Aerodynamical constants.
        Ax = 0.25;
        Ay = 0.25;
        Az = 0.25;
    end
    
    properties(Access = private)
        animeDrone           % hgtransform handle for the Drone Animation
        animeDroneBody       % hgtransform handle for the Drone Body
        animeDronePropeller  % hgtransform handle for the Drone Propeller
        animeDroneFrame      % hgtransform handle for the Drone Frame
    end
    
    methods
        %% Drone Construction Function
        function obj = robotDrone(ax, ~, x0)
            % Constructor for the Drone simulation object.
            % Inputs:
            %   ax - Axes handle where the animation will be drawn
            %   Ts - Simulation time step
            %   x0 - Initial state
            obj.xState = x0;
            % Init Motors Speed at nominal speed
            obj.omegaMotor =  ones([4, 1]).*obj.wn;
            % Create drone graphic
            obj.initAnimation(ax);
        end
        
        %% Initialize Graphic Animation for Drone
        function initAnimation(obj, ax)
            % Initialize the drone graphic in a given 3D axes.
            % Draw coordinate axes at the center of mass of the drone.
            %
            % Inputs:
            %   ax - 3D axes handle for drawing the drone

            % The x-axis is drawn in red, y-axis in green 
            % and z-axis in blue.

            % Length of the coordinate axes for visualization
            axisLength = 0.5;  
            % Draw x-axis (red)
            hx = line(ax, [0, axisLength], [0, 0], [0, 0],...
                'Color', 'r', 'LineWidth', 2);
            % Draw y-axis (green)
            hy = line(ax, [0, 0], [0, axisLength], [0, 0],...
                'Color', 'g', 'LineWidth', 2);
            % Draw z-axis (blue)
            hz = line(ax, [0, 0], [0, 0], [0, axisLength],...
                'Color', 'b', 'LineWidth', 2);
            
            % Create a transformation group to allow for easy updates
            obj.animeDrone          = hgtransform('Parent', ax);
            obj.animeDroneFrame     = hgtransform('Parent', ax);
            obj.animeDroneBody      = hgtransform('Parent', ax);
            obj.animeDronePropeller = hgtransform('Parent', ax);

            % Extract Drone 3D Model
            fv = stlread('media\DroneBody.stl');
            f = fv.ConnectivityList;
            v = fv.Points;

            patch('Faces', f, 'Vertices', v,...
                'FaceColor', [1 1 1], 'EdgeColor',...
                'none', 'Parent', obj.animeDroneBody);

            % Extract Propellers 3D Model
            fv = stlread('media\DronePropeller.stl');
            f = fv.ConnectivityList;
            v = fv.Points;

            patch('Faces', f, 'Vertices', v,...
                'FaceColor', [0.6 0.6 0.6], 'EdgeColor',...
                'none', 'Parent', obj.animeDronePropeller);

            % Apply ilumination
            camlight('headlight');
            
            % Set the Coordinate Frame
            set(hx, 'Parent', obj.animeDroneFrame);
            set(hy, 'Parent', obj.animeDroneFrame);
            set(hz, 'Parent', obj.animeDroneFrame);

            % Set the created graphic objects as children of the 
            % hgtransform
            set(obj.animeDroneBody,...
                'Parent', obj.animeDrone);
            set(obj.animeDronePropeller,...
                'Parent', obj.animeDrone);
            set(obj.animeDroneFrame,...
                'Parent', obj.animeDrone);
        end
        
        %% Update Graphic Animation for Drone
        function updateAnimation(obj, x)
            % Update the drone graphic based on the new state x.
            % Inputs:
            %   x - New state [x; y; z; yaw]
            
            % The transformation is composed of a translation (x, y, z)
            % and a rotation about the z-axis (yaw).
            psi   = x(7);  % yaw
            theta = x(8);  % pitch
            phi   = x(9);  % roll
            position = [x(1), x(2), x(3)];

            % Translation to fix STL Import for DroneBody.
            stl_body = [-0.25, 0.0, -0.30];

            % Translation to fix STL Import for DronePropeller.
            stl_propeller = [-0.345, 0.275, -0.365];

            % Translation to fix DroneFrame.
            frame = [0.0, 0.0, 0.15];
            
            %% Create combined transformation

            % Body Drone Transform
            H_Body = makehgtform(...
                'xrotate', pi/2,...
                'yrotate', pi/2,...
                'translate', stl_body,...
                'scale', 0.00125);
            obj.animeDroneBody.Matrix = H_Body;

            % Drone Propellers Transform
            H_Propeller = makehgtform(...
                'xrotate', pi/2,...
                'yrotate', pi/2,...
                'translate', stl_propeller,...
                'scale', 0.00125);
            obj.animeDronePropeller.Matrix = H_Propeller;

            % Drone Frame Transform
            H_Frame = makehgtform(...
                'translate', frame);
            obj.animeDroneFrame.Matrix = H_Frame;

            % General Drone Transform
            H = makehgtform(...
                'translate', position,...
                'zrotate', psi,...
                'yrotate', theta,...
                'xrotate', phi);
            obj.animeDrone.Matrix = H;
        end
        
        %% Drone Dynamics Model
        function dx = modelDynamics(obj, ~, x, u)
            % modelDynamics: Drone dynamics model with gyroscopic 
            % effects from rotor inertia.
            %
            % Inputs:
            %   t - Current time (not used in this implementation)
            %   x - State vector:
            %         x(1)  = x position [m]
            %         x(2)  = y position [m]
            %         x(3)  = z position [m]
            %         x(4)  = x velocity [m/s]
            %         x(5)  = y velocity [m/s]
            %         x(6)  = z velocity [m/s]
            %         x(7)  = yaw angle (psi) [rad]
            %         x(8)  = pitch angle (theta) [rad]
            %         x(9)  = roll angle (phi) [rad]
            %         x(10) = roll rate (p) [rad/s]
            %         x(11) = pitch rate (q) [rad/s]
            %         x(12) = yaw rate (r) [rad/s]
            %
            %   u - Control input vector:
            %         u(1) = Total thrust T
            %         u(2) = Roll torque tau_phi
            %         u(3) = Pitch torque tau_theta
            %         u(4) = Yaw torque tau_psi
            %
            % Output:
            %   dx - State derivative vector
        
            dx = zeros(12, 1);
            
            % Position derivatives
            dx(1) = x(4);  % dx/dt = x velocity
            dx(2) = x(5);  % dy/dt = y velocity
            dx(3) = x(6);  % dz/dt = z velocity
        
            % Velocity derivatives (linear accelerations)
            % The horizontal accelerations depend on the thrust u(1) 
            % and the drone's orientation.
            dx(4) = (1/obj.m) * ( (cos(x(9))*cos(x(7))*sin(x(8))...
                + sin(x(9))*sin(x(7)))*u(1) ) - obj.Ax*x(4);
            dx(5) = (1/obj.m) * ( (cos(x(9))*sin(x(8))*sin(x(7))...
                - cos(x(7))*sin(x(9)))*u(1) ) - obj.Ay*x(5);
            % In vertical direction, subtract gravity and add drag:
            dx(6) = -obj.g + (1/obj.m) * (cos(x(9))*cos(x(8))*u(1))...
                - obj.Az*x(6);
            
            % Euler angles derivatives (kinematics)
            % Yaw (psi) derivative:
            dx(7) = (sin(x(9))/cos(x(8)))*x(11)...
                + (cos(x(9))/cos(x(8)))*x(12);
            % Pitch (theta) derivative:
            dx(8) = cos(x(9))*x(11) - sin(x(9))*x(12);
            % Roll (phi) derivative:
            dx(9) = x(10) + (sin(x(8))/cos(x(8)))*sin(x(9))*x(11)...
                + (sin(x(8))/cos(x(8)))*cos(x(9))*x(12);
            
            % Angular velocity derivatives (rotational accelerations)
            % Let p = x(10) (roll rate), q = x(11) (pitch rate), 
            % r = x(12) (yaw rate). Compute the rotor speed difference 
            % (omegaGamma) from the four motor speeds:
            % omegaGamma = omega1 - omega2 + omega3 - omega4.
            omegaGamma = obj.omegaMotor(1) - obj.omegaMotor(2)...
                + obj.omegaMotor(3) - obj.omegaMotor(4);
            
            % Roll angular acceleration (p_dot):
            % Include the gyroscopic term due to rotor inertia:
            %   p_dot = ((Iy - Iz)/Ix)*q*r - (Ir/Ix)*q*omegaGamma
            %           + (1/Ix)*tau_phi
            dx(10) = ((obj.Iy - obj.Iz)/obj.Ix)*x(11)*x(12) ...
                     - (obj.Ir/obj.Ix)*x(11)*omegaGamma + (1/obj.Ix)*u(2);
            
            % Pitch angular acceleration (q_dot):
            %   q_dot = ((Iz - Ix)/Iy)*p*r + (Ir/Iy)*p*omegaGamma 
            %           + (1/Iy)*tau_theta
            dx(11) = ((obj.Iz - obj.Ix)/obj.Iy)*x(10)*x(12) ...
                     + (obj.Ir/obj.Iy)*x(10)*omegaGamma + (1/obj.Iy)*u(3);
            
            % Yaw angular acceleration (r_dot):
            % In this formulation, no additional gyroscopic 
            % term is added for yaw.
            dx(12) = ((obj.Ix - obj.Iy)/obj.Iz)*x(10)*x(11)...
                + (1/obj.Iz)*u(4);
        end
        
        %% Open-Loop Control for Drone
        function u = manualControl(obj, joystick_axes)
            % controlDrone - Computes open-loop control inputs for 
            % the drone based on joystick input.
            %
            % Inputs:
            %   joystick_axes - Processed axis values from the joystick.
            %
            % Outputs:
            %               u - Control input vector 
            
            obj.omegaMotor = ones([4, 1]).*obj.wn;
            
            % Total Trust Manual Speed Control
            obj.omegaMotor = obj.omegaMotor...
                + (obj.wn*(joystick_axes(3)/10));

            % Roll Manual Speed Control
            obj.omegaMotor(1) = obj.omegaMotor(1)...
                - (obj.wn*(joystick_axes(1)/100));
            obj.omegaMotor(2) = obj.omegaMotor(2)...
                - (obj.wn*(joystick_axes(1)/100));
            obj.omegaMotor(3) = obj.omegaMotor(3)...
                + (obj.wn*(joystick_axes(1)/100));
            obj.omegaMotor(4) = obj.omegaMotor(4)...
                + (obj.wn*(joystick_axes(1)/100));

            % Pitch Manual Speed Control
            obj.omegaMotor(1) = obj.omegaMotor(1)...
                - (obj.wn*(joystick_axes(2)/100));
            obj.omegaMotor(2) = obj.omegaMotor(2)...
                + (obj.wn*(joystick_axes(2)/100));
            obj.omegaMotor(3) = obj.omegaMotor(3)...
                + (obj.wn*(joystick_axes(2)/100));
            obj.omegaMotor(4) = obj.omegaMotor(4)...
                - (obj.wn*(joystick_axes(2)/100));

            % Yaw Manual Speed Control
            obj.omegaMotor(1) = obj.omegaMotor(1)...
                - (obj.wn*(joystick_axes(4)/100));
            obj.omegaMotor(2) = obj.omegaMotor(2)...
                + (obj.wn*(joystick_axes(4)/100));
            obj.omegaMotor(3) = obj.omegaMotor(3)...
                - (obj.wn*(joystick_axes(4)/100));
            obj.omegaMotor(4) = obj.omegaMotor(4)...
                + (obj.wn*(joystick_axes(4)/100));

            % The allocation matrix relates the squared motor speeds to 
            % the total thrust and torques:
            %
            % [    T    ]   [  k       k       k       k   ] [ omega1^2 ]
            % [ tau_phi ] = [ -l*k   -l*k     l*k     l*k  ] [ omega2^2 ]
            % [tau_theta]   [ -l*k    l*k     l*k    -l*k  ] [ omega3^2 ]
            % [ tau_psi ]   [ -b      b      -b       b    ] [ omega4^2 ]

            % Force-Torque Matrix
            forcetorqueMatrix =...
                [[       obj.k,        obj.k,       obj.k,        obj.k];
                 [-obj.l*obj.k, -obj.l*obj.k, obj.l*obj.k,  obj.l*obj.k];
                 [-obj.l*obj.k,  obj.l*obj.k, obj.l*obj.k, -obj.l*obj.k];
                 [      -obj.b,        obj.b,      -obj.b,        obj.b]];

            % Forces applied to Drone
            u = forcetorqueMatrix*(obj.omegaMotor.^2);
        end

        %% Attitude Control for assisted flight
        function u = attitudeAltitudeControl(obj, xd)
            % attitudeControl: PD attitude control for roll and pitch 
            % with gravity compensation
            %
            % Inputs:
            %           obj - Drone object containing parameters
            %             x - State vector, where it is assumed that:
            %                x(8)  = Pitch angle (theta) [rad]
            %                x(9)  = Roll angle (phi) [rad]
            %                x(10) = Roll angular velocity (p) [rad/s]
            %                x(11) = Pitch angular velocity (q) [rad/s]
            %
            % Output:
            %   motorSpeeds - Vector of motor speeds.
            
            %% Extract state variables
            % Altitude (Z value)
            z      = obj.xState(3);

            % Linear Z Velocity
            z_dot  = obj.xState(6);

            % Yaw, pitch, and roll angles
            psi    = obj.xState(7);  % Yaw (ψ) [rad]
            theta  = obj.xState(8);  % Pitch (θ) [rad]
            phi    = obj.xState(9);  % Roll (φ) [rad]

            % Angular velocities
            p = obj.xState(10);      % Roll angular velocity [rad/s]
            q = obj.xState(11);      % Pitch angular velocity [rad/s]
            r = obj.xState(12);      % Yaw angular velocity [rad/s]
            
            %% Define desired angles (hover: 0 rad)
            psi_des   = xd(7);
            theta_des = xd(8);
            phi_des   = xd(9);
            
            %% Compute errors and apply PD control law for torques
            error_phi   = phi - phi_des;
            error_theta = theta - theta_des;
            error_psi   = wrapToPi(psi - psi_des);
            
            % PD gains (to be tuned experimentally)
            Kp_phi   = 5;       Kd_phi   = 0.1;
            Kp_theta = 5;       Kd_theta = 0.1;
            Kp_psi   = 0.2;    Kd_psi   = 0.1;
            
            % Compute control torques using PD law
            tau_phi   = - (Kp_phi * error_phi     + Kd_phi * p);
            tau_theta = - (Kp_theta * error_theta + Kd_theta * q);
            tau_psi   = - (Kp_psi * error_psi     + Kd_psi * r);

            %% Altitude PD Control
            error_z      = xd(3) - z;
            % Desired vertical velocity is zero in hover
            error_z_dot  = xd(6) - z_dot;

            % PD gains for altitude control
            Kp_z = 3;       Kd_z = 6;

            % Additional thrust required to correct altitude error
            T_alt = obj.m * (Kp_z * error_z + Kd_z * error_z_dot);
            
            %% Gravity compensation
            % To maintain hover, the effective vertical thrust must 
            % satisfy:
            %   T_effective = T * cos(phi) * cos(theta) = m * g
            % Therefore, compute the total thrust T required:
            T = (obj.m * obj.g + T_alt) / (cos(phi) * cos(theta));
            
            %% Compute motor commands
            % The allocation matrix relates the squared motor speeds to 
            % the total thrust and torques:
            %
            % [    T    ]   [  k       k       k       k   ] [ omega1^2 ]
            % [ tau_phi ] = [ -l*k   -l*k     l*k     l*k  ] [ omega2^2 ]
            % [tau_theta]   [ -l*k    l*k     l*k    -l*k  ] [ omega3^2 ]
            % [ tau_psi ]   [ -b      b      -b       b    ] [ omega4^2 ]
            
            % The desired input vector is:
            u = [T; tau_phi; tau_theta; tau_psi];
            
            % Force-Torque Matrix
            forcetorqueMatrix =...
                [[       obj.k,        obj.k,       obj.k,        obj.k];
                 [-obj.l*obj.k, -obj.l*obj.k, obj.l*obj.k,  obj.l*obj.k];
                 [-obj.l*obj.k,  obj.l*obj.k, obj.l*obj.k, -obj.l*obj.k];
                 [      -obj.b,        obj.b,      -obj.b,        obj.b]];

            % Solve for the squared motor speeds:
            omegaSq = forcetorqueMatrix \ u;

            % Ensure non-negative values and compute the motor speeds:
            omegaSq = max(omegaSq, 0);
            % Saturate Motor Speed for pseudo realistic values
            obj.omegaMotor = min(sqrt(omegaSq), obj.maxOmega);
            % Forces applied to Drone
            u = forcetorqueMatrix*(obj.omegaMotor.^2);
        end

        function xd = positionXYControlSaturated(obj, xd)
            % positionXYControlSaturated: Outer-loop PD controller 
            % for XY position with saturation of desired pitch and roll 
            % references, incorporating a transformation from the global 
            % to the body frame.
            %
            % Inputs:
            %   xd - Desired state vector (12x1) where:
            %         xd(1:3) are the desired global positions,
            %         xd(4:5) are the desired global velocities,
            %         xd(8) and xd(9) will be updated with the desired 
            %               pitch and roll.
            %
            % Outputs:
            %   xd - Updated desired state vector with:
            %         xd(8) = desired pitch angle (theta_ref) [rad],
            %         xd(9) = desired roll angle (phi_ref) [rad].
            
            % Compute position error in the global frame [ex; ey]
            error_global     = xd(1:2) - obj.xState(1:2);
            
            % Compute velocity error in the global frame [ex_dot; ey_dot]
            error_vel_global = xd(4:5) - obj.xState(4:5);
            
            % Get the current yaw (psi) from the drone's state
            psi = obj.xState(7);
            
            % Compute the rotation matrix from global to body frame 
            % using current yaw:
            % [e_x_body; e_y_body] = [cos(psi)   sin(psi);
            %                         -sin(psi)  cos(psi)] * [ex; ey]
            R = [cos(psi)  sin(psi); 
                -sin(psi)  cos(psi)];
            
            % Transform the errors from the global frame into the drone's
            % body frame

            % [ex_body; ey_body]
            error_body     = R * error_global;
            % [ex_dot_body; ey_dot_body]
            error_vel_body = R * error_vel_global; 
            
            % Define PD gains for the position controller
            Kp_x = 6;  Kd_x = 8;
            Kp_y = 6;  Kd_y = 8;
            
            % Compute the desired horizontal accelerations in the
            % body frame a_x_body and a_y_body (in m/s^2)
            a_x_body = Kp_x * error_body(1) + Kd_x * error_vel_body(1);
            a_y_body = Kp_y * error_body(2) + Kd_y * error_vel_body(2);
            
            % Map the desired body-frame accelerations to pitch and roll 
            % references using the small-angle approximation:
            % For a quadrotor:
            %   a_x_body ≈  g * theta_ref  ==> theta_ref =  a_x_body / g
            %   a_y_body ≈ -g * phi_ref    ==> phi_ref   = -a_y_body / g
            theta_ref =  a_x_body / obj.g;
            phi_ref   = -a_y_body / obj.g;
            
            % Saturate the desired angles to a maximum (e.g., ±10°)
            max_angle = deg2rad(10);  % Convert 10 degrees to radians
            theta_ref = max(min(theta_ref, max_angle), -max_angle);
            phi_ref   = max(min(phi_ref, max_angle), -max_angle);
            
            % Update the desired state vector with these attitude 
            % references:
            xd(8) = theta_ref;  % desired pitch angle
            xd(9) = phi_ref;    % desired roll angle
        end
    end
end