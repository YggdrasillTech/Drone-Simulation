function results = RunSingleDrone(varargin)
    % RunSingleDrone: Single-drone simulation, manual and automatic.
    %
    % Successor to Project_2/DroneSimulation.m.
    %
    % Usage:
    %   RunSingleDrone()                     % defaults
    %   RunSingleDrone('mode', 'manual')     % start in manual
    %   RunSingleDrone('input', 'keyboard')  % force keyboard
    %   r = RunSingleDrone('duration', 20);  % capture the log
    %
    % Options:
    %      input - 'auto' | 'joystick' | 'keyboard'   (default 'auto')
    %       mode - 'auto' | 'manual'                  (default 'auto')
    %   duration - Simulated seconds                  (default 30)
    %     layout - Rotor geometry, 'x' or 'plus'
    %      theme - 'dark' or 'light'
    %   realTime - Pace against the wall clock        (default true)
    %   headless - Run with no figure, for batch studies
    %
    % Controls:
    %   Arrows  pitch / roll     W,S  climb / descend
    %   A,D     yaw              M    toggle manual / auto
    %   R       reset            Q    quit
    %
    % Output:
    %   results - Flight log struct (t, x, e, omega, mode)

    % Put src/ on the path, resolved relative to this file so the
    % simulation runs from any working directory. A bootstrap cannot
    % live in the folder it bootstraps, so this is the one line that
    % stays at the root.
    addpath(fullfile(fileparts(mfilename('fullpath')), 'src'));

    opt = parseOptions(varargin{:});

    %% Parameters and configuration
    P   = sim_droneParams(opt.layout);
    cfg = sim_config();
    cfg.duration = opt.duration;
    cfg.realTime = opt.realTime;

    % Simulation initial state for the Drone
    x0 = [0; 0; 0;
          0; 0; 0;
          -pi/2; pi/7; pi/6;
          0; 0; 0];       % Initial State

    % Desired State. Note z is POSITIVE up: the model of Eq. (21) has z
    % increasing upwards, so Project_2's desired z of -1 commanded the
    % drone underground, below both the ground plane and the [0 10]
    % limits of its own 3D axes.
    ref = struct('pos', [3; 2; 4], 'vel', zeros(3, 1), 'yaw', 0);

    %% Objects
    drone      = sim_QuadrotorPlant(P, x0);
    controller = sim_CascadeController(P);
    cmd        = sim_PilotInput.neutral();

    if opt.headless
        gui = [];  view3d = [];  pilot = [];
    else
        gui    = sim_Window('full', 'Drone Simulation', opt.theme);
        view3d = sim_DroneGraphic(gui.axSim);
        view3d.update(x0);
        pilot  = sim_PilotInput(gui.wnd, opt.input);

        % Flush the first render before the clock starts. Building a
        % maximised figure with seven subplots costs the better part of
        % a second, and if that lands inside the timed loop the clock
        % opens already half a second behind and immediately resyncs.
        drawnow;
    end

    % Drive Mode signals if the robot is in Manual or Automated control
    pilotMode = double(strcmpi(opt.mode, 'manual'));

    %% Logging
    % Preallocated. Project_2 grew xf = [xf, x0] inside the loop, which
    % reallocates and copies the whole array every step - O(n^2), and
    % steadily slower the longer the run, which is exactly backwards for
    % a real-time loop needing a predictable per-step budget.
    nLog = 0;
    cap  = ceil(min(cfg.duration, 600) * cfg.physicsRate ...
        / cfg.logDivider) + 2;
    logT = zeros(1, cap);
    logX = zeros(12, cap);
    logE = zeros(12, cap);
    logW = zeros(4, cap);
    logM = zeros(1, cap);

    %% Simulation Loop
    %
    % Every subsystem is driven off the physics step counter, so the
    % relationship between control rate and dynamics rate is stated once
    % in sim_config and holds exactly. The inner attitude loop has a
    % divider of 1: it runs on EVERY physics step, in lockstep with the
    % plant, which is what Project_2/3 did not do.
    clk = sim_FixedStepClock(cfg);
    clk.start();
    quitting = false;

    while ~quitting && clk.simTime < cfg.duration
        if ~opt.headless && ~gui.isOpen()
            break;
        end

        nSteps = clk.stepsDue();

        for k = 1:nSteps
            t = clk.simTime;

            %% Pilot input
            if ~opt.headless && clk.due(cfg.CH_INPUT, cfg.inputDivider)
                cmd = pilot.read();

                if cmd.modeToggle
                    pilotMode = 1 - pilotMode;
                    controller.reset();
                end
                if cmd.reset
                    drone.xState = x0;
                    drone.omegaMotor = ones(4, 1) * P.wHover;
                    controller.reset();
                    gui.clearTrace();
                end
                quitting = quitting || cmd.quit;
            end

            %% Outer loop: position -> attitude references
            if clk.due(cfg.CH_OUTER, cfg.outerDivider)
                dtOuter = cfg.dt * cfg.outerDivider;
                if pilotMode == 1
                    controller.manualLoop(cmd, drone.xState, dtOuter);
                else
                    controller.outerLoop(drone.xState, ref, dtOuter);
                end
            end

            %% Inner loop at the physics rate, then the plant
            uDesired = controller.innerLoop(drone);
            drone.advance(t, cfg.dt, uDesired, cfg.groundLevel);

            clk.advance();

            %% Log
            if cfg.logData && clk.due(cfg.CH_LOG, cfg.logDivider)
                nLog = nLog + 1;
                logT(nLog)    = clk.simTime;
                logX(:, nLog) = drone.xState;
                logE(:, nLog) = trackingError(drone.xState, ref, ...
                    controller);
                logW(:, nLog) = drone.omegaMotor;
                logM(nLog)    = pilotMode;
            end
        end

        %% Graphics, on their own independent schedule
        if ~opt.headless && nSteps > 0
            if clk.due(cfg.CH_RENDER, cfg.renderDivider)
                view3d.update(drone.xState);
            end

            if clk.due(cfg.CH_PLOT, cfg.plotDivider)
                gui.update(clk.simTime, drone.xState, ...
                    trackingError(drone.xState, ref, controller), ...
                    drone.omegaMotor, pilotMode);
            end

            if clk.due(cfg.CH_STATUS, cfg.statusDivider)
                gui.setStatus(statusText(clk, pilotMode, drone, pilot));
            end

            % limitrate skips frames the renderer cannot keep up with,
            % instead of queueing them the way 'update' does.
            drawnow limitrate;
        end

        clk.idle();
    end

    %% Cleanup and report
    if ~opt.headless
        pilot.close();
    end

    s = clk.report();
    fprintf(['Simulated %.2f s in %.2f s of wall clock ' ...
             '(%.2fx real time, %d steps, %d resyncs).\n'], ...
        s.simTime, s.wallTime, s.rtFactor, s.steps, s.resyncs);

    %% Results
    results = struct();
    if nLog > 0
        idx = 1:nLog;
        results = struct('t', logT(idx), 'x', logX(:, idx), ...
            'e', logE(:, idx), 'omega', logW(:, idx), 'mode', logM(idx));

        if ~opt.headless
            plotResults(results, opt.theme);
        end
    end
end

%% Tracking error for logging and plots
function e = trackingError(x, ref, controller)
    % trackingError: State error against the ACTIVE references.
    %
    % Project_2 computed  error = x0 - xd  after the position controller
    % had already overwritten xd(8) and xd(9) with its internal tilt
    % references, so the "angle error" plot silently mixed the user's
    % reference with a controller internal. Here attitude error is taken
    % against the controller's references, position against the user's.

    e = zeros(12, 1);
    e(1:3) = x(1:3) - ref.pos;
    e(4:6) = x(4:6) - ref.vel;
    e(7:9) = sim_wrapAngle(x(7:9) - ...
        [controller.refPsi; controller.refTheta; controller.refPhi]);
end

%% Heads-up text
function s = statusText(clk, pilotMode, drone, pilot)
    % statusText: Summary drawn over the 3D view.

    if pilotMode == 1
        modeStr = 'MANUAL';
    else
        modeStr = 'AUTO';
    end

    x = drone.xState;

    s = sprintf([ ...
        't = %6.2f s   %-6s   %s\n' ...
        'pos  [%6.2f %6.2f %6.2f] m\n' ...
        'att  [%6.1f %6.1f %6.1f] deg  (psi theta phi)\n' ...
        'rotor %4.0f %4.0f %4.0f %4.0f rad/s'], ...
        clk.simTime, modeStr, pilot.name, ...
        x(1), x(2), x(3), ...
        rad2deg(x(7)), rad2deg(x(8)), rad2deg(x(9)), ...
        drone.omegaMotor(1), drone.omegaMotor(2), ...
        drone.omegaMotor(3), drone.omegaMotor(4));
end

%% Summary figure
function plotResults(r, themeName)
    % plotResults: Six panels summarising the run.
    %
    % Project_2 opened six separate figure windows, each with its own
    % hand-written xlabel/ylabel/title/grid block. One tiled figure is
    % easier to read side by side and is one window to close.

    % No point drawing more points than a screen can resolve.
    step = max(1, round(numel(r.t) / 3000));
    k = 1:step:numel(r.t);
    t = r.t(k);

    spec = { ...
        r.x(1:3, k),            'XYZ Position',     'Position [m]',   {'x','y','z'}; ...
        rad2deg(r.x(7:9, k)),   'Attitude',         'Angle [deg]',    {'\psi','\theta','\phi'}; ...
        r.x(4:6, k),            'XYZ Velocity',     'Velocity [m/s]', {'v_x','v_y','v_z'}; ...
        rad2deg(r.x(10:12, k)), 'Angular Velocity', 'Rate [deg/s]',   {'p','q','r'}; ...
        r.e(1:3, k),            'Position Error',   'Error [m]',      {'e_x','e_y','e_z'}; ...
        r.omega(:, k),          'Motor Speeds',     'Speed [rad/s]',  {'\omega_1','\omega_2','\omega_3','\omega_4'} };

    f = figure('Name', 'Flight Results', 'NumberTitle', 'off');
    try
        theme(f, themeName);
    catch
    end

    for i = 1:size(spec, 1)
        subplot(3, 2, i);
        plot(t, spec{i, 1}, 'LineWidth', 1.2);
        axis tight;  grid on;
        title(spec{i, 2});
        xlabel('Time [s]');
        ylabel(spec{i, 3});
        legend(spec{i, 4}, 'Location', 'best', ...
            'Orientation', 'horizontal');
    end
end

%% Option parsing
function opt = parseOptions(varargin)
    % parseOptions: Name-value options with defaults.

    p = inputParser;
    p.addParameter('input',    'auto', @(v) ischar(v) || isstring(v));
    p.addParameter('mode',     'auto', @(v) ischar(v) || isstring(v));
    p.addParameter('layout',   'x',    @(v) ischar(v) || isstring(v));
    p.addParameter('theme',    'dark', @(v) ischar(v) || isstring(v));
    p.addParameter('duration', 30,     @(v) isnumeric(v) && v > 0);
    p.addParameter('realTime', true,   @(v) islogical(v) || isnumeric(v));
    p.addParameter('headless', false,  @(v) islogical(v) || isnumeric(v));
    p.parse(varargin{:});

    opt = p.Results;
    opt.input    = char(opt.input);
    opt.mode     = char(opt.mode);
    opt.layout   = char(opt.layout);
    opt.theme    = char(opt.theme);
    opt.realTime = logical(opt.realTime);
    opt.headless = logical(opt.headless);
end
