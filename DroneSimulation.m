%% Drone Simulation Code
clear; clc; close all;

function runDroneSimulation()
    %% Initialization of Parameters and Objects
    % Create a joystick object
    joy = sim3d.io.Joystick();
    
    % Simulation initial state for the Drone
    x0 = zeros([12, 1]); % Initial State
    xd = zeros([12, 1]); % Desire State
    t0 = 0;              % Initial simulation time
    Ts = 0.008;          % Simulation time step (~120 Hz)

    %% Create Drone GUI
    DroneGUI = droneGUI();

    %% Create Robot Handler
    simHandler = DroneGUI.getSimHandler();

    %% Create Drone Handler
    % Instantiate the drone with internal parameters 
    Drone = robotDrone(simHandler, Ts, x0);
    % Drive Mode signals is the robot is in Manual or Automated control
    pilotMode = 0;
    
    pause(1.0);  % Allow the figure to initialize properly
    
    %% Start rate control
    r = robotics.Rate(1/Ts);
    
    %% Simulation Loop
    while ishandle(DroneGUI.wnd)
        % Read joystick input and update control signals
        [a, b, p] = read(joy);
        [a, b, ~] = postProcessJoystick(a, b, p);
        
        % Exit simulation if the start button (button 8) is pressed
        if b(8)
            break;
        end
        
        % Reset Drone to Initial State
        if b(3)
            % Reset Drone State
            x0 = zeros([12, 1]);
            % Reset Desired State
            xd = zeros([12, 1]);
        end

        % If A button is pressed
        % (button 1) return to origin.
        if b(1) && (pilotMode == 0)
            pilotMode = 1;
            % Init State, time is conserved for the simulation
            xd = zeros([12, 1]);
            xd(3) = 5;
        end 
        
        %% Compute control inputs using open-loop mapping for the Drone.
        % Mapping:
        %   a(2) - forward/backward
        %   a(1) - lateral (sideways)
        %   a(3) - vertical (up/down)
        %   a(4) - yaw control
        % u = Drone.manualControl(a);

        %% Autonomous return to desire state
        if pilotMode == 1
            error = norm(rmse(xd, x0), 2);
            % Verify error or button to exit autonomous mode.
            if  (error > 0.02 ) && ~b(2)
                xd = Drone.positionXYControlSaturated(xd);
            else
                % Return to Manual Mode
                pilotMode = 0;
            end
        else
            %% Compute control inputs using closed-loop assited flight.
            % Compute Desire State with Joystick Inputs
            xd = joystickToDesiredState(a, xd, Ts);
        end
        
        %% Compute Control Forces
        u = Drone.attitudeAltitudeControl(xd);
        
        %% Perform one simulation step using Euler integration (via ode1)
        [t0, x0] = simulateStep(t0, x0, u, @Drone.modelDynamics, Ts);

        % Normalize yaw to the range [-pi, pi]
        x0(7) = wrapToPi(x0(7));
        x0(8) = wrapToPi(x0(8));
        x0(9) = wrapToPi(x0(9));
        
        %% Update the drone's state and animation
        Drone.xState = x0;
        Drone.updateAnimation(x0);
        DroneGUI.updateGraphics(t0, x0, Drone.omegaMotor, pilotMode);
        
        % Force the graphics to refresh immediately
        drawnow update;
        
        % Maintain the simulation time step by rate control
        waitfor(r);
    end
    
    %% Cleanup
    % Delete the joystick object if valid
    if exist('joy', 'var') && isvalid(joy)
        delete(joy);
    end
    % Close the figure if it is still open
    if ishandle(DroneGUI.wnd)
        pause(1.0); % Pause briefly
        delete(DroneGUI.wnd);
    end
end

%% Simulation Step Function using Euler integration (ode1)
function [t, x] = simulateStep(t_current, x_current, u_current, f, Ts)
    % Integrate the drone's motion over a short time interval using
    % a simple Euler method.
    [t_int, x_int] = ode1(@(t, x) f(t, x, u_current), t_current,...
        Ts/50, t_current + Ts, x_current);
    t = t_int(end);
    x = x_int(:, end);
end

%% Post Process Joystick Function 
function [a, b, p] = postProcessJoystick(a0, b0, p0)
    % postProcessJoystick: Processes raw joystick input.
    %
    % Input:
    %   a0 - 5 floating-point axis values.
    %   b0 - 10 binary button values.
    %   p0 - 1 integer point-of-view value.
    %
    % Output:
    %    a - Processed axis values with deadzone applied and 
    %        rescaled to [-1, 1].
    %    b - Processed button values; only returns 1 on a rising 
    %        edge (transition 0->1).
    %    p - The point-of-view value (unchanged).

    %% Deadzone parameter for the axes
    deadzone = 0.1;
    
    a = a0;
    b = b0;
    p = p0;
    
    % Process Axis Values: set to zero if within deadzone and 
    % rescale linearly.
    idx = abs(a) >= deadzone;
    a(~idx) = 0;
    a(idx) = sign(a(idx)) .* ((abs(a(idx)) - deadzone) / (1 - deadzone));
    a = -a; % Adjust direction
    
    % Process Button Values: only detect rising edge.
    persistent prevButtons;
    if isempty(prevButtons)
        prevButtons = zeros(size(b));
    end
    newPress = (b == 1) & (prevButtons == 0);
    prevButtons = b;
    b = double(newPress);
end

function xd =...
    joystickToDesiredState(joystick_axes, xd, Ts)
    % Integrates joystick inputs to update the desired
    % altitude and yaw references using ode1.
    %
    % Inputs:
    %   joystick_axes - Vector of joystick axis values.
    %              xd - Previous desired altitude (m).
    %              Ts - Integration time step (s).
    %
    % Outputs:
    %              xd - New Desired State
    
    %% Define scaling factors
    % [m/s] per unit of joystick input for altitude
    altitudeScaling = 2;  
    % [rad/s] per unit of joystick input for yaw
    yawScaling      = 2;       
    
    % Initial state for the integrator: [z_des; yaw_des]
    initState = [xd(3); xd(7)];
    
    % Define the derivative function for the desired state
    % The derivative is constant over Ts, determined by the current 
    % joystick input.
    f = @(t, z) [joystick_axes(3) * altitudeScaling; 
                 joystick_axes(4) * yawScaling];
    
    % Use ode1 to integrate over the time step Ts.
    % Here, we use a small integration step (e.g., Ts/10) for 
    % better accuracy.
    dt = Ts / 10;
    t0 = 0;
    t_end = Ts;
    [~, x_int] = ode1(f, t0, dt, t_end, initState);
    
    % Get the final integrated state
    finalState = x_int(:, end);
    xd(3) = finalState(1);
    xd(7) = wrapToPi(finalState(2));
    xd(8) = joystick_axes(2)/5;
    xd(9) = -joystick_axes(1)/5;
end

%% Run the Simulation
runDroneSimulation();