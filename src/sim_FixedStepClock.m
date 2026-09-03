classdef sim_FixedStepClock < handle
    % sim_FixedStepClock: Fixed-timestep pacing for the simulation loop.
    %
    % Replaces the timing model of Project_2/3:
    %
    %     r = robotics.Rate(1/Ts);
    %     while ...
    %         u = control(...);              % once per 8 ms
    %         [t, x] = simulateStep(...);    % 50 Euler sub-steps
    %         drawnow update;                % every 4th iteration
    %         waitfor(r);                    % hope we kept up
    %     end
    %
    % which had three problems. The controller ran once per 8 ms while
    % the plant advanced in 0.16 ms sub-steps, so the loop was really a
    % 125 Hz zero-order hold around a fast plant. waitfor(robotics.Rate)
    % needs the Robotics System Toolbox and silently drops periods it
    % cannot meet, so simulated and wall-clock time diverged with nothing
    % reporting it. And every subsystem was decimated off one counter,
    % tying the render rate to the control rate.
    %
    % The replacement is the standard fixed-timestep accumulator: the
    % wall clock decides how many physics steps are owed, each step is
    % exactly dt, and every subsystem has its own divider.

    properties (SetAccess = private)
        dt              % [s] fixed physics step
        simTime = 0     % [s] simulated time
        stepCount = 0   % Physics steps executed
        lag = 0         % [s] how far behind the wall clock we are
        resyncs = 0     % Times the backlog was abandoned
    end

    properties (Access = private)
        cfg
        wallStart
        % Simulated time credited by resyncs. Simulated time is
        % stepCount*dt PLUS this, so it is exact in dt multiples and a
        % resync still sticks - see the note in resync().
        timeOffset = 0
        % Step count at which each subsystem channel last fired. -Inf so
        % every channel is due on the first iteration.
        lastFired = -inf(1, 8)
    end

    properties (Constant, Access = private)
        % Past this backlog, catching up is hopeless and pretending
        % otherwise just runs the simulation flat out for seconds.
        RESYNC_THRESHOLD = 0.5;   % [s]
    end

    methods
        %% Construction
        function obj = sim_FixedStepClock(cfg)
            % Constructor.
            %
            % Input:
            %   cfg - Config struct from sim_config()

            obj.cfg = cfg;
            obj.dt  = cfg.dt;
        end

        %% Start timing
        function start(obj)
            % start: Zero the clock and begin pacing.
            obj.wallStart  = tic;
            obj.simTime    = 0;
            obj.stepCount  = 0;
            obj.timeOffset = 0;
            obj.lag        = 0;
            obj.resyncs    = 0;
            obj.lastFired  = -inf(1, 8);
        end

        %% Physics steps owed right now
        function n = stepsDue(obj)
            % stepsDue: Number of physics steps to run this iteration.
            %
            % Free-running mode always returns 1 and the loop goes as
            % fast as the machine allows. Real-time mode returns however
            % many whole steps the wall clock has moved past, capped so
            % a stall cannot snowball.
            %
            % Output:
            %   n - Steps to execute before rendering again

            if ~obj.cfg.realTime
                n = 1;
                return;
            end

            wall = toc(obj.wallStart) * obj.cfg.timeScale;
            obj.lag = wall - obj.simTime;

            if obj.lag > obj.RESYNC_THRESHOLD
                obj.resync(wall);
                n = 1;
                return;
            end

            n = sim_saturate(floor(obj.lag / obj.dt), ...
                0, obj.cfg.maxStepsPerFrame);
        end

        %% Commit one physics step
        function advance(obj)
            % advance: Register that one dt has been integrated.
            %
            % Simulated time is rebuilt from the step count rather than
            % accumulated, so it never drifts - but it must include the
            % resync offset. Without that term a resync sets simTime
            % forward and the very next advance() throws the jump away,
            % the lag reappears instantly, and the clock resynchronises
            % again every iteration. That is what produced 105 resyncs
            % in a 30 s run that was otherwise keeping real time.

            obj.stepCount = obj.stepCount + 1;
            obj.simTime   = obj.timeOffset + obj.stepCount * obj.dt;
        end

        %% Subsystem scheduling
        function tf = due(obj, channel, divider)
            % due: Whether a subsystem running every 'divider' physics
            % steps should run now. Firing is recorded, so this both
            % asks and commits.
            %
            % Deliberately NOT mod(stepCount, divider) == 0, which is
            % what Project_2/3 used. Once the loop runs several physics
            % steps per iteration - the whole point of the accumulator -
            % the step counter jumps, and a mod test can step straight
            % over its own multiple. A render on a divider of 8 whose
            % counter goes 6, 10, 14 never fires at all.
            %
            % Inputs:
            %   channel - Subsystem id, see the CH_* fields of sim_config
            %   divider - Step divider, 1 means every step
            %
            % Output:
            %        tf - true if the subsystem is due

            tf = (obj.stepCount - obj.lastFired(channel)) >= divider;

            if tf
                obj.lastFired(channel) = obj.stepCount;
            end
        end

        %% Yield when ahead of schedule
        function idle(obj)
            % idle: Give time back to the OS and the graphics queue when
            % the simulation is running ahead of the wall clock.
            %
            % Without this the loop spins at 100% CPU polling the clock
            % between steps, starving the rendering it is waiting for.

            if ~obj.cfg.realTime
                return;
            end

            ahead = obj.simTime - toc(obj.wallStart)*obj.cfg.timeScale;

            % pause() has millisecond-scale granularity, so asking for
            % less than that just adds overhead.
            if ahead > 0.002
                pause(min(ahead, 0.01));
            end
        end

        %% Timing report
        function s = report(obj)
            % report: Summary of how well real time was held.
            %
            % Output:
            %   s - Struct with simulated time, wall time, step count,
            %       real-time factor and resync count

            wallElapsed = toc(obj.wallStart);

            s = struct( ...
                'simTime',  obj.simTime, ...
                'wallTime', wallElapsed, ...
                'steps',    obj.stepCount, ...
                'rtFactor', obj.simTime / max(wallElapsed, eps), ...
                'resyncs',  obj.resyncs);
        end
    end

    methods (Access = private)
        %% Abandon an unrecoverable backlog
        function resync(obj, wall)
            % resync: Jump simulated time to the wall clock and count it,
            % so the run reports honestly that it could not keep up.
            %
            % Credited through timeOffset rather than by writing simTime
            % directly, because advance() rebuilds simTime from the step
            % count and would otherwise undo it.

            obj.timeOffset = obj.timeOffset + (wall - obj.simTime);
            obj.simTime    = wall;
            obj.lag        = 0;
            obj.resyncs    = obj.resyncs + 1;
        end
    end
end
