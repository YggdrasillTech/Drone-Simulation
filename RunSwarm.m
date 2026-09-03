function RunSwarm(varargin)
    % RunSwarm: Multi-drone consensus formation.
    %
    % Successor to Project_3/DroneSimulation.m, on the same simulation
    % core as the single-drone case: one shared mesh, one clock, one
    % controller class, one plant class.
    %
    % The pilot flies the FORMATION and the swarm keeps its shape around
    % that moving goal, which exercises the manual input path and the
    % formation law together. Press M for manual; in AUTO the formation
    % holds a fixed point.
    %
    % Usage:
    %   RunSwarm()
    %   RunSwarm('numDrones', 5, 'input', 'keyboard')
    %
    % Options:
    %   numDrones - Swarm size (default 9)
    %       input - 'auto' | 'joystick' | 'keyboard'
    %    duration - Simulated seconds (default 60)
    %      layout - Rotor geometry, 'x' or 'plus'
    %       theme - 'dark' or 'light'
    %
    % Controls: as RunSingleDrone, applied to the formation.

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

    n    = opt.numDrones;
    lead = n;                    % lead drone is the last one

    %% sim_FormationPlanner
    % Randomised slot assignment, leader kept on the central slot.
    others = setdiff(1:n, lead);
    order  = zeros(1, n);
    order(lead)   = lead;
    order(others) = others(randperm(numel(others)));

    planner = sim_FormationPlanner(boxFormation(n), lead, order);

    %% Objects
    gui   = sim_Window('sim', 'Multi Drone Simulation', opt.theme);
    pilot = sim_PilotInput(gui.wnd, opt.input);
    cmd   = sim_PilotInput.neutral();

    x0 = gridStart(n);

    % Every view shares one cached, decimated copy of the STL data - see
    % sim_DroneGraphic.loadMeshes. Project_3 read and reduced both STL files
    % inside every robotDrone constructor, so a 9-drone run parsed 51k
    % triangles eighteen times before the first frame.
    colors = lines(n);
    drones      = cell(1, n);
    controllers = cell(1, n);
    views       = cell(1, n);

    for i = 1:n
        drones{i}      = sim_QuadrotorPlant(P, x0(:, i));
        controllers{i} = sim_CascadeController(P);

        if i == lead
            c = [1 1 1];         % leader in white
        else
            c = colors(i, :);
        end

        views{i} = sim_DroneGraphic(gui.axSim, c);
        views{i}.update(x0(:, i));
    end

    % Flush the first render before the clock starts, so figure
    % construction does not count as simulation lag.
    drawnow;

    %% Working buffers, allocated once rather than per step
    pos = zeros(3, n);
    vel = zeros(3, n);
    targets = zeros(3, n);

    leaderTarget = [0; 0; 5];
    pilotMode = 0;

    %% Simulation Loop
    clk = sim_FixedStepClock(cfg);
    clk.start();
    quitting = false;

    while ~quitting && gui.isOpen() && clk.simTime < cfg.duration
        nSteps = clk.stepsDue();

        for k = 1:nSteps
            t = clk.simTime;

            %% Pilot input
            if clk.due(cfg.CH_INPUT, cfg.inputDivider)
                cmd = pilot.read();
                if cmd.modeToggle
                    pilotMode = 1 - pilotMode;
                end
                quitting = quitting || cmd.quit;
            end

            %% Formation planning
            % Runs on the outer-loop schedule, ONCE for the whole swarm.
            % Project_3 recomputed the neighbour graph and the confidence
            % matrix inside its per-drone loop, so the same O(N^2) result
            % was rebuilt N times per step.
            if clk.due(cfg.CH_SWARM, cfg.outerDivider)
                dtOuter = cfg.dt * cfg.outerDivider;

                for i = 1:n
                    pos(:, i) = drones{i}.xState(1:3);
                    vel(:, i) = drones{i}.xState(4:6);
                end

                if pilotMode == 1
                    leaderTarget = moveGoal(leaderTarget, cmd, P, dtOuter);
                end

                targets = planner.desiredPositions(pos, vel, leaderTarget);

                for i = 1:n
                    controllers{i}.outerLoop(drones{i}.xState, ...
                        struct('pos', targets(:, i), ...
                               'vel', zeros(3, 1), 'yaw', 0), dtOuter);
                end
            end

            %% Inner loop and plant, per drone, at the physics rate
            for i = 1:n
                uDesired = controllers{i}.innerLoop(drones{i});
                drones{i}.advance(t, cfg.dt, uDesired, cfg.groundLevel);
            end

            clk.advance();
        end

        %% Graphics
        if nSteps > 0 && clk.due(cfg.CH_RENDER, cfg.renderDivider)
            for i = 1:n
                views{i}.update(drones{i}.xState);
            end

            % Trace the leader only; N traces would swamp the view.
            gui.update(clk.simTime, drones{lead}.xState, ...
                zeros(12, 1), drones{lead}.omegaMotor, pilotMode);

            drawnow limitrate;
        end

        if nSteps > 0 && clk.due(cfg.CH_STATUS, cfg.statusDivider)
            gui.setStatus(statusText(clk, pilotMode, pos, targets, pilot));
        end

        clk.idle();
    end

    %% Cleanup and report
    pilot.close();

    s = clk.report();
    fprintf(['Simulated %.2f s in %.2f s of wall clock ' ...
             '(%.2fx real time, %d drones, %d resyncs).\n'], ...
        s.simTime, s.wallTime, s.rtFactor, n, s.resyncs);
end

%% Move the formation goal from pilot input
function g = moveGoal(g, cmd, P, dt)
    % moveGoal: Translate stick deflection into motion of the formation
    % centroid, in the inertial frame, kept inside the drawn volume.

    speed = 3.0;   % [m/s] at full deflection

    g(1) = sim_saturate(g(1) + cmd.pitch*speed*dt, -8, 8);
    g(2) = sim_saturate(g(2) + cmd.roll *speed*dt, -8, 8);
    g(3) = sim_saturate(g(3) + cmd.throttle*P.maxClimbRate*dt, 1, 9);
end

%% sim_FormationPlanner shape
function xs = boxFormation(n)
    % boxFormation: The eight-corner box plus a centre point used by
    % Project_3, generalised to any swarm size. The centre slot is last,
    % because that is the leader's.
    %
    % Input:
    %   n - Number of drones
    %
    % Output:
    %  xs - 3xn formation template

    template = [ 0 -2  4;   0 -2  6;   0  2  4;   0  2  6;
                -2  0  4;  -2  0  6;   2  0  4;   2  0  6;
                 0  0  5 ].';

    if n <= size(template, 2)
        idx = [1:n-1, size(template, 2)];
        xs  = template(:, idx);
        return;
    end

    % Larger swarms: a ring per layer, so the shape stays sensible.
    xs = zeros(3, n);
    for i = 1:n
        layer = floor((i - 1) / 8);
        ang   = 2*pi * mod(i - 1, 8) / 8;
        xs(:, i) = [3*cos(ang); 3*sin(ang); 4 + 2*layer];
    end
end

%% Initial positions
function x0 = gridStart(n, spacing)
    % gridStart: Scatter the swarm over a jittered, shuffled grid.
    %
    % Inputs:
    %         n - Number of drones
    %   spacing - Distance between grid cells (default 4.0)
    %
    % Output:
    %        x0 - 12xn matrix of initial states

    if nargin < 2
        spacing = 4.0;
    end

    side = ceil(sqrt(n));
    pos  = zeros(3, n);
    c    = 0;

    for i = 1:side
        for j = 1:side
            if c < n
                c = c + 1;
                pos(:, c) = [ ...
                    (i - ceil(side/2))*spacing + rand()*2 - 1;
                    (j - ceil(side/2))*spacing + rand()*2 - 1;
                    1 + rand()*7];
            end
        end
    end

    x0 = zeros(12, n);
    x0(1:3, :) = pos(:, randperm(n));
end

%% Heads-up text
function s = statusText(clk, pilotMode, pos, targets, pilot)
    % statusText: sim_FormationPlanner summary drawn over the 3D view.
    %
    % Two numbers say whether it is working: RMS distance to the
    % commanded slot (has it converged) and the closest pair (is it safe).

    if pilotMode == 1
        modeStr = 'MANUAL - flying the formation';
    else
        modeStr = 'AUTO   - formation holds';
    end

    rms = sqrt(mean(sum((targets - pos).^2, 1)));

    n = size(pos, 2);
    gap = inf;
    for i = 1:n
        for j = i+1:n
            gap = min(gap, norm(pos(:, i) - pos(:, j)));
        end
    end

    s = sprintf([ ...
        't = %6.2f s   %s\n' ...
        'device %s\n' ...
        'drones %d   formation RMS %5.2f m   min gap %5.2f m'], ...
        clk.simTime, modeStr, pilot.name, n, rms, gap);
end

%% Option parsing
function opt = parseOptions(varargin)
    % parseOptions: Name-value options with defaults.

    p = inputParser;
    p.addParameter('numDrones', 9,      @(v) isnumeric(v) && v >= 2);
    p.addParameter('input',     'auto', @(v) ischar(v) || isstring(v));
    p.addParameter('layout',    'x',    @(v) ischar(v) || isstring(v));
    p.addParameter('theme',     'dark', @(v) ischar(v) || isstring(v));
    p.addParameter('duration',  60,     @(v) isnumeric(v) && v > 0);
    p.parse(varargin{:});

    opt = p.Results;
    opt.input  = char(opt.input);
    opt.layout = char(opt.layout);
    opt.theme  = char(opt.theme);
end
