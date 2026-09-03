function C = sim_config()
    % sim_config: Rates and options for the fixed-step simulation loop.
    %
    % Every rate in the simulator is stated once, here, in Hz - and the
    % control rate is locked to the physics rate rather than being an
    % accident of how the loop happens to be written.
    %
    % Project_2/3 ran the controller once per 8 ms outer iteration and
    % then sub-stepped the plant 50 times inside that iteration with
    % Euler. The controller therefore saw a 125 Hz zero-order hold while
    % the plant advanced at 6.25 kHz. Here the inner loop runs in
    % lockstep with the plant.
    %
    % Output:
    %   C - Struct of simulation settings.

    %% Core rates
    % Physics integration step. RK4 at 500 Hz is both more accurate and
    % cheaper than forward Euler at ten times the rate; RunTests
    % measures both.
    C.physicsRate = 500;                    % [Hz]
    C.dt          = 1 / C.physicsRate;      % [s]

    % Subsystem dividers, in physics steps. A divider of 1 means the
    % subsystem runs on every single step, in lockstep with the plant.
    C.innerDivider  = 1;      % attitude + altitude + mixer -> 500 Hz
    C.outerDivider  = 5;      % position loop               -> 100 Hz
    C.inputDivider  = 10;     % pilot input poll            ->  50 Hz
    C.logDivider    = 5;      % telemetry logging           -> 100 Hz
    C.renderDivider = 10;     % 3D animation                ->  50 Hz
    C.plotDivider   = 50;     % scrolling telemetry plots   ->  10 Hz
    C.statusDivider = 100;    % heads-up text               ->   5 Hz

    % Channel ids. sim_FixedStepClock records the last firing step per channel, so
    % each subsystem needs a stable slot to be counted in.
    C.CH_INPUT  = 1;
    C.CH_OUTER  = 2;
    C.CH_RENDER = 3;
    C.CH_PLOT   = 4;
    C.CH_STATUS = 5;
    C.CH_LOG    = 6;
    C.CH_SWARM  = 7;

    %% Real-time behaviour
    % When true the loop is paced against the wall clock with an
    % accumulator, so simulated time tracks real time even if a frame
    % takes long. When false it runs as fast as it can, which is what
    % batch analysis and the tests want.
    C.realTime  = true;
    C.timeScale = 1.0;      % 2.0 = twice real time

    % Upper bound on physics steps executed per wall-clock iteration.
    % Without a cap, a long stall makes the loop ask for more steps than
    % it can run, which makes the next iteration longer still - the
    % classic "spiral of death". The cap must still be generous enough
    % to absorb one slow render: at 500 Hz a 40 ms frame owes 20 steps,
    % so a cap of 8 (the first value tried here) could never catch up
    % and the run bled lag until it resynchronised, over and over.
    C.maxStepsPerFrame = 40;

    %% Run control
    C.duration = 30;        % [s] simulated time, Inf to run until closed
    C.logData  = true;

    %% Ground
    % The paper's model has no ground: a drone commanded downwards keeps
    % going. A hard floor stops the state running away in manual flight.
    C.groundLevel = 0;      % [m], -Inf to disable

    %% Appearance
    C.theme = 'dark';       % 'dark' | 'light'
end
